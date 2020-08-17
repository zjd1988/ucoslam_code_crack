
#include "system.h"
#include <opencv2/highgui.hpp>
int main( int  argc , char**  argv )
{
    if (argc<3){cerr<<"Usage: in out "<<endl;return -1;}
    ucoslam::System tw;
    tw.readFromFile(argv [1]);
    tw.TheMap->saveToPcd(argv[2],0.1);
    return 0;

}

