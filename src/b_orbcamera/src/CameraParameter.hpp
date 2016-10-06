#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"
class CameraParameter
{
public:
    CameraParameter(){};
    ~CameraParameter(){};

    double fxl, fyl, cxl, cyl;
    double fxr, fyr, cxr, cyr;
    double rot[9];
    double t[3];

    void loadCameraParams(char * uri)
    {
        char tmpbuf[128];
        double cx, cy, fx, fy, zero;
        double tx, ty, tz;
        std::ifstream inf;

        inf.open(uri, std::ios::in);
        inf.getline(tmpbuf, sizeof(tmpbuf));
        inf >> fxl >> zero >> cxl >>
            zero >> fyl >> cyl >>
            zero >> zero >> zero;

        inf.getline(tmpbuf, sizeof(tmpbuf));
        inf.getline(tmpbuf, sizeof(tmpbuf));
        inf >> fxr >> zero >> cxr >>
            zero >> fyr >> cyr >>
            zero >> zero >> zero;

        inf.getline(tmpbuf, sizeof(tmpbuf));
        inf.getline(tmpbuf, sizeof(tmpbuf));
        inf >> zero; rot[0] = zero;
        inf >> zero; rot[1] = zero;
        inf >> zero; rot[2] = zero;
        inf >> zero; rot[3] = zero;
        inf >> zero; rot[4] = zero;
        inf >> zero; rot[5] = zero;
        inf >> zero; rot[6] = zero;
        inf >> zero; rot[7] = zero;
        inf >> zero; rot[8] = zero;

        inf.getline(tmpbuf, sizeof(tmpbuf));
        inf.getline(tmpbuf, sizeof(tmpbuf));
        inf >> tx; t[0] = tx;
        inf >> ty; t[1] = ty;
        inf >> tz; t[2] = tz;

        inf.close();
    }

    bool checkFile(char * file)
    {
        bool r = false;

        if (access(file, 0) != -1)
        {
            std::cout << "-- find " << file << std::endl << std::endl;
            r = true;
        }
        else
        {
            std::cout << "-- Didn't find " << file << std::endl << std::endl;
        }

        return r;
    }

    void calcALLMat(double *mat)
	{
		cv::Mat RK_Mat = (cv::Mat_<double>(4,4) << fxr, 0.0, cxr, 0.0,
				0.0, fyr, cyr, 0.0,
				0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 1.0);

		cv::Mat R2L_Mat = (cv::Mat_<double>(4,4) << rot[0], rot[1], rot[2], t[0],
				rot[3], rot[4], rot[5], t[1],
				rot[6], rot[7], rot[8], t[2],
					   0.0, 0.0, 0.0, 1.0);

		cv::Mat LK_Mat = (cv::Mat_<double>(4,4) << fxl, 0.0, cxl, 0.0,
				0.0, fyl, cyl, 0.0,
				0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 1.0);

		cv::Mat M_Mat = RK_Mat*R2L_Mat;
		cv::Mat All_Mat = M_Mat*LK_Mat.inv();
		mat[0] = All_Mat.at<double>(0, 0);
		mat[1] = All_Mat.at<double>(0, 1);
		mat[2] = All_Mat.at<double>(0, 2);
		mat[3] = All_Mat.at<double>(0, 3);
		mat[4] = All_Mat.at<double>(1, 0);
		mat[5] = All_Mat.at<double>(1, 1);
		mat[6] = All_Mat.at<double>(1, 2);
		mat[7] = All_Mat.at<double>(1, 3);
		mat[8] = All_Mat.at<double>(2, 0);
		mat[9] = All_Mat.at<double>(2, 1);
		mat[10] = All_Mat.at<double>(2, 2);
		mat[11] = All_Mat.at<double>(2, 3);
		mat[12] = All_Mat.at<double>(3, 0);
		mat[13] = All_Mat.at<double>(3, 1);
		mat[14] = All_Mat.at<double>(3, 2);
		mat[15] = All_Mat.at<double>(3, 3);

		//std::cout << "RK_Mat" << std::endl << RK_Mat << std::endl;
		//std::cout << "LK_Mat" << std::endl << LK_Mat << std::endl;
		//std::cout << "R2L_Mat" << std::endl << R2L_Mat << std::endl;
		//std::cout << "All_Mat" << std::endl << All_Mat << std::endl;
	}

    void calcALLMatDefault(double *mat)
	{
		double default_mat[16]=
		{   1.02599 ,    	0.0037541,	    0.790151,  	7197.62,
		   -0.0109156 ,     1.02855,    	-25.1729,   14079.8,
		   -1.94943e-005 , 	-7.9034e-006,	1.0082,  	-1.86347,
		   0 ,            	0,          	0   ,     	1};
		for(int i = 0; i < 16; i++)
		{
			mat[i] = default_mat[i];
		}
	}




private:

};
