#include <iostream>
#include <string>
#include <slamBase.h>
#include <sstream>

using namespace std;

int main()
{
    FramePair readFrame( int index );
}


FramePair readFrame( int index )
{
    stringstream ss;
    ss << "rgb" << index << ".png";
    string filename;
    ss >> filename;

    FramePair result;
    result.rgb = cv::imread(filename);

    ss.clear();
    filename.clear();

    ss << "depth" << index << ".png";
    ss >> filename;
    result.depth = cv::imread(filename, -1);

    return result;
}

