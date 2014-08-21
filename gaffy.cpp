// gaffy.cpp
#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>

void printUsage()
{
    std::cout << "Throwing errors!!\nAdd the arguments file1.csv file2.csv after your program." << std::endl << std::flush;
    std::exit(1);
}

std::string openCSV(std::ifstream &ifs, char * name)
{
    ifs.open (name, std::ifstream::in);

    std::string contents = "";
    char c = ifs.get();

    while (ifs.good()) {
        contents.push_back(c);
        c = ifs.get();
    }
    ifs.close();
    return contents;
}

std::string getData(std::string content)
{
    std::stringstream   findNewLines(content);
    std::string         value;

    int count = 0;

    getline(findNewLines, value,'\n');                  // process first line

    std::stringstream   ls(value);
    while(getline(ls, value,';')){                      // find line header mass,kg ....
        count++;
        if(value == "mass,kg,inst-value,0,0,0")
            break;
    }

    getline(findNewLines, value,'\n');                  // process second line

    std::stringstream ls2(value);
    while(getline(ls2, value,';')){
        count--;
        if(count < 0)                                   // break loop when found data
            break;
    }
    return value;
}

std::string modValues(std::string a, std::string b)
{
    std::string kg = getData(a);

    std::stringstream ss;

    std::stringstream   findNewLines(b);
    std::string         value;

    int count, tempcount = 0;

    getline(findNewLines, value,'\n');                  // process first line

    std::stringstream   ls(value);
    while(getline(ls, value,';')){                      // find line header mass,kg ....
        tempcount++;
        ss << value << ";";
        if(value == ",energy")
            count = tempcount;
    }

    ss<<"\n";
    getline(findNewLines, value,'\n');                  // process second line

    std::stringstream ls2(value);
    while(getline(ls2, value,';')){
        count--;
        if(count != 1){                                   // break loop when found data
            ss << value << ";";
        }else
            ss << kg << ";";
    }
    return ss.str();
}

void save(std::string content, std::string filename)
{
    std::ofstream myfile;

    myfile.open(filename.c_str());
    myfile << content;
    myfile.close();

    return;
}

int main()
{
    std::vector<std::string> files;

    std::ifstream ifs;

    files.push_back(openCSV(ifs, "/home/mark/hydro/file2.csv"));         // read files to strings
    files.push_back(openCSV(ifs, "/home/mark/hydro/file1.csv"));         // read files to strings

    std::string output = modValues(files[0], files[1]);                      // modify the values of file1.csv

    save(output, "file3.csv");

    std::cout << "Done" << std::endl;

    return 0;
}
