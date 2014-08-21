// emoji.cpp
#include<stdio.h>
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
#include <string>

class iphoneGame
{
public:
    iphoneGame(bool scale)
    {
        half = scale;
        //window size//
        this->mainHeight    = 17;     //40
        this->mainWidth     = 25;    //22

        iconW = 33; iconH = 33;

        readInData();
    }

    void readInData()
    {
        std::ifstream ifs;
        ifs.open ("/home/mark/Desktop/emojiGame/map.txt", std::ifstream::in);

        char c = ifs.get();

        while (ifs.good()) {
            mainMapSTR.push_back(c);
            c = ifs.get();
        }
        ifs.close();

        char file[1000];
        for(int i = 0; i < 880; i++)        // hardcoding is bad!
        {
            sprintf(file, "/home/mark/Desktop/emojiGame/emoji/%d.png", i);
            emojis.push_back(cv::imread(file, CV_LOAD_IMAGE_COLOR));
        }
    }

    void execute()
    {
        mainWindow = cv::Mat::zeros(mainWidth*iconW, mainHeight*iconH, CV_8UC3);

        // game related code goes here


        tile();
        visualise();
    }

    void tile() {
        int noobHardCodedMap[] = {
        0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,7,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

        int tileID = 269;
        //int tileID = 0;

        for(int i = 0; i < mainHeight; i++) {
            for(int j = 0; j < mainWidth ; j++) {

                emojis[noobHardCodedMap[i*17+j]].copyTo(mainWindow(cv::Rect(i*iconW,j*iconH,iconW,iconH)));
            }
        }
    }

    void visualise()            // some sort of a bug here?
    {
        if(half)
            resize(mainWindow, mainWindow, cv::Size(mainWindow.cols/2, mainWindow.rows/2));
        cv::imshow("asdasd",mainWindow);
        cv::waitKey(10033);
        return;
    }


private:
    bool half;
    std::string mainMapSTR;
    int mainHeight, mainWidth;
    int iconW, iconH;
    std::vector <cv::Mat> emojis;
    std::vector <int> mainMap;
    cv::Mat mainWindow;
    int temp[425];
};

int main()
{
    /*
    int noobHardCodedMap[425] = {
    0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
    */

    iphoneGame x(true);


/*
    while(true)
    {
       //x.execute();
        x.visualise();
        ////////////////////////////////// hang
        std::cout << "." << std::flush;
        cv::waitKey(1);
    }
*/
    for(int i = 0; i < 10; i++)
    {
        x.execute();
    }

    return 0;
}
