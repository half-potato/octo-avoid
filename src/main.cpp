#include <iostream>
#include <math.h>
#include <vector>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace octomap;

struct VoxelInfo {
	public:
		double x; // absolute, not relative coordinates
		double y;
		double z;
		double dist;	// Distance
		double lambda;	// Angle size
		double az;	// Azimuth
		double el;	// Elevation
		double oc;	// Occupancy certanity
		double l;	// Minimum distance to voxel
		double voxelSize;
		VoxelInfo(double x, double y, double z, double dist, double oc) : x(x), y(y), z(z), dist(dist), oc(oc) {}
		VoxelInfo(double x, double y, double z, double dist, double lambda, double az, double el, double oc, double l) : x(x), y(y), z(z), dist(dist), lambda(lambda), az(az), el(el), oc(oc), l(l) {}
		~VoxelInfo() {}
};

typedef vector< VoxelInfo > VoxelList;
typedef vector<vector<vector<double>>> christmastree;

// 5 stages
//   1st
//       Explore a bounding box around the vehicle. The location of voxels is not stored to reduce memory.
// Then calculate the azimuth and elevation angles for the voxel. These are coordinates. (radians) 
cv::Point2d coordsToAngles(double ox, double oy, double oz, double vx, double vy, double vz, double alpha) {
    //azimuth = floor(1/alpha * arctan((xi - x0) / (yi - y0))) where alpha is the resolution of the histogram, xi, yi are the coords of the voxel, and x0, y0 are the vehicle center point
	double az = floor(1.0/alpha * atan((vx - ox) / (vy - oy)));
    //elevation = floor(1/alpha * arctan((zi - z0) / sqrt((xi-x0)**2 + (yi-y0)**2)))
	double el = floor(1.0/alpha * atan((vz - oz) / sqrt(pow(vx - ox, 2) + pow(vy - oy, 2))));
	cv::Point2d result(az, el);
	return result;
}

VoxelList exploreSphere(OcTree octo, cv::Point3d origin, double ws, int depth) {
	VoxelList list;
	OcTreeKey min; 
	bool ok0;
	int d0 = depth;
	do {
		if (d0 < 1) {
			cout << "Why the fuck is your octomap empty?" << endl;
			return list;
		}
		ok0 = octo.coordToKeyChecked(origin.x - ws/2, origin.y - ws/2, origin.z - ws/2, min);
		d0--;
	} while (!ok0);

	//OcTreeKey min = octo.coordToKey(origin.x - ws/2, origin.y - ws/2, origin.z - ws/2);
	OcTreeKey max;
	bool ok1;
	int d1 = depth;
	do {
		if (d1 < 1) {
			cout << "Why the fuck is your octomap empty?" << endl;
			return list;
		}
		ok1 = octo.coordToKeyChecked(origin.x + ws/2, origin.y + ws/2, origin.z + ws/2, d1, max);
		d1--;
	} while (!ok1);

	//OcTreeKey max = octo.coordToKey(origin.x + ws/2, origin.y + ws/2, origin.z + ws/2);
	for (OcTree::leaf_bbx_iterator i = octo.begin_leafs_bbx(min, max, depth); i != octo.end_leafs_bbx(); ++i) {
		if(i != NULL) {
			if(i->getOccupancy() > 0) {
				double x = i.getX(), y = i.getY(), z = i.getZ();
				double dist = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
				if (dist < ws) {
					VoxelInfo v(x, y, z, dist, i->getOccupancy());
					v.voxelSize = i.getSize();
					list.push_back(v);
				}
			}
		}
	}
	return list;
}
// Alpha is the maximum angle that would remain in the same cell of the histogram in radians
// Use the radius of the robot to enlarge the voxel, this determines the angles
// lambda = floor(1/alpha * arcsine((r+s+v)/d)) where r is the robot radius, s is the safety radius, v is the voxel size, d is the distance to the voxel, and lambda is the voxel angle size
// l = d - (r+s+v) where l is the minimum distance to the voxel

vector <vector <double> > primaryHistogram(VoxelList *data, double robotRadius, double safetyRadius, double alpha, double ws, cv::Point3d origin) {
	// a-b((ws-1)/2)**2 = 1 //ratio for the histogram, only the ratio matters
	double b = 5;
	double a = 1 + b*pow( (ws-1) / 2, 2);
	cout << "b: " << b << "a: " << a << endl;
	cout << 1/alpha << endl;
	vector <vector <double> > histogram;
	int eres = (int)ceil(M_PI_2 / alpha);
	int zres = (int)ceil(M_PI_2 / alpha);
	//Pregen data
	for (auto i=data->begin(); i!=data->end(); ++i) {
		i->l = i->dist - (robotRadius+safetyRadius+i->voxelSize);
		i->lambda = (1/alpha * asin((robotRadius+safetyRadius+i->voxelSize)/i->dist));
		cv::Point2d cds = coordsToAngles(origin.x, origin.y, origin.z, i->x, i->y, i->z, alpha);
		i->az = cds.x;
		i->el = cds.y;
		//i->lambda = 
		//cout << 1/alpha * asin((robotRadius+safetyRadius+i->voxelSize)/i->dist) << endl;
	}
	//  histogram(e,z) = SUM(if (e is a component of [elevation-lambda/alpha, elevation+lambda/alpha] and z is a component of [azimuth-lambda/alpha, azimuth+lambda/alpha] then 
	//                       eo**2 * (a-b*l) 
	//                    else
	//                       0))
	cout << (a - b*data->back().l) << endl;;
	for (int z=0; z<zres; z++) {
		histogram.push_back({});
		for (int e=0; e<eres; e++) {
			// implement histogram(e, z)
			double sum = 0;
			for (auto i=data->begin(); i!=data->end(); ++i) {
				//Figure out if this voxel lies within this cell
				if(( (e > (i->el - i->lambda / alpha)) && (e < (i->el + i->lambda / alpha)) ) &&
				   ( (z > (i->az - i->lambda / alpha)) && (z < (i->az + i->lambda / alpha)) ) ) {
					sum += pow(i->oc, 2);// * (a - b*i->l);	// Add weight
				} // Essentially add 0
			}
			histogram.back().push_back(sum);
		}
	}
	return histogram;
}
//   3rd stage
//       Adding more voxels to include the turning circle
//       First, calculate the origins of the turning circles
//          dXr = r_r * sin(theta)
//          dXr = -r_l * sin(theta)
//          dYr = r_r * cos(theta)
//          dYr = -r_l * cos(theta)
//          Where r_r is the turning radius on the right, r_l is the turning radius on the left, and theta is the angle of the turn
//       Second, check active cells in the turning circles
//           If they lie in the circles, compare the distance to the safety range r+s+v and the diameter of the respective circle
//           dr = sqrt((dxr - dx(i))**2 + (dyr - dx(i))**2) where i represents the voxel index and r represents the right hand turning circle
//           dl = sqrt((dxl - dx(i))**2 + (dyl - dx(i))**2) where i represents the voxel index and l represents the right hand turning circle
//           they lie within the circles if dr < r_r+r+s+v or dl < r_l+r+s+v
//       Third, check climbing motion
//           Find the maximum boundaries the robot could climb to get the voxels in that area
//           This uses the climbing motion constant f using the altitude difference and the turning distance t
//               t = (2*pi*r*2*az) / 360 where r is probably the distance to the voxel and az azimuth
//               f = (zi - z0) / t where zi is the current altitude and z0 is the previous altitude
//           taz = (2*pi*r*2*betaz*alpha) / 360 where betaz is the unreachable azimuth angle, alpha is the resolution of the histogram, and taz is the unreachable
//           zaz = f*taz wheer zaz is the unreachable altitude
//           laz = sqrt(r**4 - 2*r**2*cos(270 - 2*az) where laz is a variable to make equations cleaner
//           betae = 1/alpha * arctan(zaz / laz) where betae is the unreachable elevation angle
//       Add this info to the histogram
//   Fourth stage, generate the histogram
//       This stage reduces information to create a 2D binary polar histogram based on the 2D primary polar histogram
//       This accomplished by thresholding the data using the variables tlow and thigh, which are different for each cell because not all cells have the same size
//       1 for above thigh, 0 for below tlow, and histogram_primary(previous Bz, Bp) where Bp is probably previous Be because it is using the value next to the point
//       These parameters should also be different depending on speed
//           That meant that the computer was not certain enough of the position
//   Fifth stage, path detection and selection
//       Finds available paths by moving a window around the binary polar histogram
//           If all elements in the window are equal to 0, it is open
//           Wrap this search near the boundaries
//       Selects a path with the lowest path weight
//           Three path weights are calculated
//               The first is based on the angular difference between the desired direction and the path direction
//               The second is the difference between the current robot rotation and the path direction
//               The third is the difference between the last path direction and the current one
//               A vector u is used to select the desired path (u1, u2, u3)
//               ki = u1*delta(v, kt) + u2*delta(v, theta/alpha) + u3*delta(v, k_(i-1)) where v is the candidate direction, theta is the rotation of the robot, the previous direction is k_(i-1), and the target angle is k_t
/*
void graph(vector< vector<double> > histogram, const char *name) {
	TCanvas canvas(name, "Histogram", 500, 500);
	Double_t rmax(1.);
	TH2D *hist = new TH2D("polarHist", "polarHist", (int)histogram.size(), 0., M_PI_2, (int)histogram.back().size(), 0., rmax);
	Double_t theta, radius;
	for (int x=0; x<(int)histogram.size(); x++) {
		theta = hist->GetXaxis()->GetBinCenter(x);
		for (int y=0; y<(int)histogram.back().size(); y++) {
			radius = histogram.at(x).at(y);
			hist->SetBinContent(x, y, radius);
		}
	}
	TH2D *dummy = new TH2D("dummy", name, 100, -rmax, rmax, 100, -rmax, rmax);
	dummy->Draw("Col");
	hist->Draw("Col2");
	cout << "Displaying" << endl;
}*/

void graph(vector< vector<double> > histogram, const char *name, int width, int height) {
	cv::Mat background = cv::Mat::zeros(width, height, CV_8UC3);
	double r_width = width / (double) histogram.size();
	double r_height = height / (double) histogram.back().size();
	for (int x=0; x<(int)histogram.size(); x++) {
		for (int y=0; y<(int)histogram.back().size(); y++) {
			cv::Point pt1(x*r_width, y*r_height);
			cv::Point pt2(x*r_width + r_width, y*r_height + r_height);
			int c = histogram.at(x).at(y) * 255;
			cv::rectangle(background, pt1, pt2, cv::Scalar(c, c, c), -1);
		}
	}
	for(;;)
	{
		cv::imshow(name, background);
		if(cv::waitKey(30) >=0) break;
	}
}

void vPrint(VoxelList v) {
	for(auto i = v.begin(); i!=v.end(); ++i) {
		cout << "P: " << i->oc << ", lambda: " << i->lambda << endl;
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "octo_avoid");
	ros::NodeHandle n("~");
	string filename;
	n.getParam("octomap", filename);
	OcTree tree(filename + ".bt");
	cv::Point3d origin(-100.0, 0.0, 0.0);
	double ws = 80.0;
	VoxelList vs = exploreSphere(tree, origin, ws, 13);
	vPrint(vs);
	vector <vector <double> > histogram = primaryHistogram(&vs, 3.0, 2.0, M_PI/20, ws, origin);
	for (auto i = histogram.begin(); i!=histogram.end(); ++i) {
		for (auto j = i->begin(); j!=i->end(); ++j) {
			cout << *j << " ";
		}
		cout << endl;
	}
	graph(histogram, "Primary histogram", 1000, 600);
}
