#include "Grid.h"
#include <boost/tokenizer.hpp>


Grid::Grid(string fname)
{
    std::string line;
    std::ifstream myfile ((fname).c_str());
    if (!myfile.is_open())
    {
        cout << "Map file " << fname << " does not exist. " << std::endl;
        exit(-1);
    }

    // cout << "*** 加载地图 ***" << std::endl;
    clock_t t = std::clock();
    size_t pos = fname.rfind('.');  // position of the file extension
    map_name = fname.substr(0, pos);  // get the name without extension
    getline (myfile, line);

    if (line[0] == 't')
    {
        // Benchmark 
        boost::char_separator<char> sep(" ");
        getline(myfile, line);
        boost::tokenizer<boost::char_separator<char>> tok(line, sep);
        boost::tokenizer<boost::char_separator<char>>::iterator beg;
        beg = tok.begin();
        beg++;
        rows = atoi((*beg).c_str()); // 读取行数
        getline(myfile, line);
        boost::tokenizer<boost::char_separator<char>> tok2(line, sep);
        beg = tok2.begin();
        beg++;
        cols = atoi((*beg).c_str()); // 读取列数
        getline(myfile, line); // 跳过 "map"

    }
    else
    {
        boost::char_separator<char> sep(",");
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
        boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
        rows = atoi((*beg).c_str());  // read number of rows
        beg++;
        cols = atoi((*beg).c_str());  // read number of cols
    }

    map.resize(cols * rows, 0);

    //DeliverGoal.resize(row*col, false);
    // 读取地图
    //int ep = 0, ag = 0;
    for (int i = 0; i < rows; i++)
    {
        getline(myfile, line);
        for (int j = 0; j < cols; j++)
        {
            int id = cols * i + j;
            if (line[j] == '@' || line[j] == 'T') // 障碍物
                map[id] = 1;
            else   // 空闲空间
                map[id] = 0;
        }
    }

    myfile.close();
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    // cout << "地图大小: " << rows << "x" << cols;
    // cout << "\t完成! (加载时间: " << runtime << " 秒)" << std::endl;
}
