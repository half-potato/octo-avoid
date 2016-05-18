#include <iostream>
#include <math.h>
#include <vector>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <TCanvas.h>
#include <TH2.h>
#include <ros/ros.h>

using namespace std;
using namespace octomap;

typedef vector <vector <vector<double > > > christmastree;
// 5 stages
//   1st
//       Explore a bounding box around the vehicle. The location of voxels is not stored to reduce memory.
christmastree getBoundingBox(OcTree octo, double x, double y, double z, double ws, double res, int searchDepth) { //where ws is the width, height, and depth
	christmastree result;
	for (double ix=(x-ws/2); ix < (x+ws/2); ix+=res) {
		result.push_back({});
		for (double iy=(y-ws/2); iy < (y+ws/2); iy+=res) {
			result.back().push_back({});
			for (double iz=(z-ws/2); iz < (z+ws/2); iz+=res) {
				point3d point(ix, iy, iz);
				OcTreeNode* node = octo.search(point, searchDepth);
				result.back().back().push_back(node->getOccupancy());
			}
		}
	}
	return result;
}
//   2nd
//       Add data to the 2d primary polar histogram, this represents a sphere
//       Determine angle using position of the voxel and vehicle center point
//       Two angles will determine range, weight will determine height
//       First, active cells will be reduced by a bounding sphere
//       ocTreeNodeWidth should be the res in the first function
christmastree getBoundingSphere(christmastree boundingBox, double ws, double voxelSize) { //where ws is the bounding radius in the same units as voxel size
	christmastree result;
	for (int x=0; x<int(boundingBox.size()); x++) {
		result.push_back({});
		for (int y=0; y<int(boundingBox.back().size()); y++) {
			result.back().push_back({});
			for (int z=0; z<int(boundingBox.back().back().size()); z++) {
				double dist = sqrt(pow((x - result.size())*voxelSize, 2) + pow((y - result.back().size())*voxelSize, 2) + pow((z - result.back().back().size())*voxelSize, 2));
				if (dist > ws) { 
					result.back().back().push_back(0.0f);
				} else {
					result.back().back().push_back(boundingBox.at(x).at(y).at(z));
				}
			}
		}
	}
	return result;
}

// Alpha is the maximum angle that would remain in the same cell of the histogram in radians
// Then calculate the azimuth and elevation angles for the voxel. These are coordinates. (radians) 
double* coordsToAngles(double ox, double oy, double oz, double vx, double vy, double vz, double alpha) {
	static double result[2] = {};
    //azimuth = floor(1/alpha * arctan((xi - x0) / (yi - y0))) where alpha is the resolution of the histogram, xi, yi are the coords of the voxel, and x0, y0 are the vehicle center point
    result[0] = floor(1.0/alpha * atan((vx - ox) / (vy - oy)));
    //elevation = floor(1/alpha * arctan((zi - z0) / sqrt((xi-x0)**2 + (yi-y0)**2)))
    result[1] = floor(1.0/alpha * atan((vz - oz) / sqrt(pow(vx - ox, 2) + pow(vy - oy, 2))));
    return result;
}
// Use the radius of the robot to enlarge the voxel, this determines the angles
// lambda = floor(1/alpha * arcsine((r+s+v)/d)) where r is the robot radius, s is the safety radius, v is the voxel size, d is the distance to the voxel, and lambda is the voxel angle size
// l = d - (r+s+v) where l is the minimum distance to the voxel
struct VoxelInfo {
	public:
		double dist;	// Distance
		double lambda;	// Angle size
		double az;	// Azimuth
		double el;	// Elevation
		double oc;	// Occupancy certanity
		double l;	// Minimum distance to voxel
		VoxelInfo(double dist, double lambda, double az, double el, double oc, double l) : dist(dist), lambda(lambda), az(az), el(el), oc(oc), l(l) {}
		~VoxelInfo() {}
};

vector <vector <double> > primaryHistogram(christmastree data, double robotRadius, double safetyRadius, double voxelSize, double alpha, double ws) {
	// a-b((ws-1)/2)**2 = 1 //ratio for the histogram, only the ratio matters
	double b = 5;
	double a = 1 + b*pow( (ws-1) / 2, 2);
	vector <vector <double> > histogram;
	vector <vector <vector <VoxelInfo> > > pregened;
	//Pregen data
	for (int x=0; x<int(data.size()); x++) {
		pregened.push_back({});
		for (int y=0; y<int(data.back().size()); y++) {
			pregened.back().push_back({});
			for (int z=0; z<int(data.back().back().size()); z++) {
				double dist = sqrt(pow((x - data.size())*voxelSize, 2) + pow((y - data.back().size())*voxelSize, 2) + pow((z - data.back().back().size())*voxelSize, 2));
				double lambda = floor(1/alpha * asin((robotRadius+safetyRadius+voxelSize)/dist));
				double *coords = coordsToAngles(0, 0, 0, x*voxelSize, y*voxelSize, z*voxelSize, alpha);
				double az = coords[0];
				double el = coords[1];
				double l = dist - (robotRadius+safetyRadius+voxelSize);
				VoxelInfo info(dist, lambda, az, el, data.at(x).at(y).at(z), l);
				pregened.back().back().push_back(info);
			}
		}
	}
	//  histogram(e,z) = SUM(if (e is a component of [elevation-lambda/alpha, elevation+lambda/alpha] and z is a component of [azimuth-lambda/alpha, azimuth+lambda/alpha] then 
	//                       eo**2 * (a-b*l) 
	//                    else
	//                       0))
	int eres = (int)ceil(M_PI / alpha);
	int zres = (int)ceil(M_PI_2 / alpha);
	for (int z=0; z<zres; z++) {
		histogram.push_back({});
		for (int e=0; e<eres; e++) {
			// implement histogram(e, z)
			double sum = 0;
			for (int x=0; x<int(pregened.size()); x++) {
				for (int y=0; y<int(pregened.back().size()); y++) {
					for (int z=0; z<int(pregened.back().back().size()); z++) {
						//Figure out if this voxel lies within this cell
						VoxelInfo *vinf = &pregened.at(x).at(y).at(z);
						if(( (e > (vinf->el - vinf->lambda / alpha)) && (e < (vinf->el + vinf->lambda / alpha)) ) &&
						   ( (e > (vinf->el - vinf->lambda / alpha)) && (e < (vinf->el + vinf->lambda / alpha)) ) ) {
							sum += pow(vinf->oc, 2) * (a - b*vinf->l);	// Add weight
						} // Essentially add 0
					}
				}
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
}

int main() {
	ros::NodeHandle n;
	string filename;
	n.getParam("octomap", filename);
	OcTree tree(filename);
	christmastree wild = getBoundingBox(tree, 0.0, 0.0, 0.0, 20.0, 1.0, 16);
	christmastree trimmed = getBoundingSphere(wild, 20.0, 1);
	vector <vector <double> > histogram = primaryHistogram(trimmed, 3.0, 2.0, 1.0, M_PI/6, 20.0);
	graph(histogram, "Primary histogram");
	for (int x=-30.0; x < 30.0; x+=2) {
		for (int y=-30.0; y < 30.0; y+=2) {
			for (int z=-30.0; z < 30.0; z+=2) {
				//point3d endpoint((double) x*0.05f, (double) y*0.05f, (double) z*0.05f);
				//tree.updateNode(endpoint, true);
				double res = 0.2f;
				point3d query ((double)x*res, (double)y*res, (double)z*res);
				OcTreeNode* result = tree.search(query, 16);
				//print_query_info(query, result);
				if (result!=NULL) { 
					printf("%f, ", result->getOccupancy());
				} else {
					cout << "NULL, ";
				}
			}
			cout << "\t";
		}
		cout << endl;
	}
}
