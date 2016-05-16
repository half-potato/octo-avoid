#include <iostream>
#include <math.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace std;
using namespace octomap;

void print_query_info(point3d query, OcTreeNode* node) {
	if (node != NULL) {
		cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
	}
	else 
		cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}

int main() {
	OcTree tree("fr_campus.bt");
	for (int x=-30.0; x < 30.0; x+=2) {
		for (int y=-30.0; y < 30.0; y+=2) {
			for (int z=-30.0; z < 30.0; z+=2) {
				//point3d endpoint((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
				//tree.updateNode(endpoint, true);
				float res = 0.2f;
				point3d query ((float)x*res, (float)y*res, (float)z*res);
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
	cout << "Hello world" << endl;
}
