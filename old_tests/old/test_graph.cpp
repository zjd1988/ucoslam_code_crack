#include <iostream>
#include <fstream>
#include "basic_types.h"
#include <graphs.h>

using namespace std;
using namespace ucoslam;

int main(void){
    try{

        ucoslam::CovisGraph cvGr;

        for (uint i = 0; i < 10; i++)
            for (uint j = i; j < 10; j++)
                 cvGr.addEdge(i,j,i+j);

        cout << cvGr;
        cout << "Neighbors of 5: " << endl;
        for (auto v : cvGr.getNeighbors(5) )
            cout << " " << v;
        cout << endl;

        // Remove node
        cvGr.removeNode(6);
        cout << "After removing nodexx: " << endl << cvGr << endl;
        cout << "Neighbors of 5: " << endl;
        for (auto v : cvGr.getNeighbors(5))
            cout << " " << v;
        cout << endl;
        // Get list of nodes
        cout << "List of nodes:" << endl;
        for (auto n : cvGr.getNodes())
        {
           cout << n << endl;
        }

        float w = cvGr.getWeight(7,1);
        cout << "Found weight: " << w << endl;

//        // Get Essential Graph
//        CovisGraph EG = cvGr.getEG(1);
//        cout << "The Essential Graph: " << endl;
//        cout << EG << endl;

        // Save and load
        ofstream file("covisgraph.data",ios::binary);
        if(!file)throw std::runtime_error("could not open file for writing: covisgraph.data");

        cvGr.toStream(file);
        file.close();
        cout << "Graph written" << endl;

        ucoslam::CovisGraph cvGr2;
        ifstream file2("covisgraph.data",ios::binary);
        if(!file)throw std::runtime_error("could not open file for reading: covisgraph.data");
        cvGr2.fromStream(file2);
        file2.close();
        cout << "*** Read: " << endl;
        cout << cvGr2;
        ofstream file3("covisgraph2.data",ios::binary);
        cvGr2.toStream(file3);

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}

