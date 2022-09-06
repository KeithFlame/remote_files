#include "map_calculator.h"
//#include "camera_parameters.h"

MapCalculator::MapCalculator(std::string parameterPath, int i, uint16_t image_width, uint16_t image_height)
    : image_width(image_width)
    , image_height(image_height)
{
    A = cv::Mat_<float>(3, 3);
    Anew = cv::Mat_<float>(3, 3);
    R = cv::Mat_<float>(3, 3);
    D = cv::Mat_<float>(1, 5);
    initMat();
    getCameraParameters(parameterPath , i);
    double_mapx.create(image_height*2, image_width*2, CV_32FC1);
    double_mapy.create(image_height*2, image_width*2, CV_32FC1);
    calcImageMap(double_mapx, double_mapy);
    updateMap();
}

void MapCalculator::updateMap(int disparity)
{
    cv::Rect roi = ROI;
    /* Although there we designate as a monocular map, we keep the disparity adjustment */
    roi.x -= disparity;

    cpu_mapx = double_mapx(roi).clone();
    cpu_mapy = double_mapy(roi).clone();

    // suppose single image size is w*h
    //int cols = cpu_mapx.cols;
    //int rows = cpu_mapx.rows;
}

void MapCalculator::getCameraParameters(std::string parameterPath,int i)
{
    std::fstream fread;
    std::string str = parameterPath + "/map/SHL01/A"+std::to_string(i+1)+".log";
    std::vector<std::string> result;
    std::cout << "开始读取文件..." << std::endl;
    //获取Ai
    fread.open(str);
    if (!fread.is_open())
    {
        std::cout << "读取文件"+str+"失败" << std::endl;
        return;
    }
    char buf[200];
    int ir = 0;
    while (fread.getline(buf,sizeof(buf)))
    {
        result = split(buf, " ,\t");
        A.at<float>(ir, 0) = atof(result[0].c_str());
        A.at<float>(ir, 1) = atof(result[1].c_str());
        A.at<float>(ir, 2) = atof(result[2].c_str());
        ir++;
    }
    std::cout << A << std::endl;
    fread.close();


    //获取Ri
    str = parameterPath + "/map/SHL01/R" + std::to_string(i + 1) + ".log";
    result.clear();
    fread.open(str);
    if (!fread.is_open())
    {
        std::cout << "读取文件" + str + "失败" << std::endl;
        return;
    }
    ir = 0;
    
    while (fread.getline(buf, sizeof(buf)))
    {
        result = split(buf, " ,\t");
        R.at<float>(ir, 0) = atof(result[0].c_str());
        R.at<float>(ir, 1) = atof(result[1].c_str());
        R.at<float>(ir, 2) = atof(result[2].c_str());
        ir++;
    }
    std::cout << R << std::endl;
    fread.close();

    //获取Di
    str = parameterPath + "/map/SHL01/D" + std::to_string(i + 1) + ".log";
    result.clear();
    fread.open(str);
    if (!fread.is_open())
    {
        std::cout << "读取文件" + str + "失败" << std::endl;
        return;
    }
    
    ir = 0;
    while (fread.getline(buf, sizeof(buf)))
    {
        result = split(buf, " ,\t");
        D.at<float>(ir, 0) = atof(result[0].c_str());
        D.at<float>(ir, 1) = atof(result[1].c_str());
        D.at<float>(ir, 2) = atof(result[2].c_str());
        D.at<float>(ir, 3) = atof(result[3].c_str());
        D.at<float>(ir, 4) = atof(result[4].c_str());
        ir++;
        std::cout << D << std::endl;
    }
    fread.close();
    if (D.rows == 1)
        D = D.t();

    //获取ROI
    str = parameterPath + "/map/SHL01/roi.log";
    result.clear();
    fread.open(str);
    if (!fread.is_open())
    {
        std::cout << "读取文件" + str + "失败" << std::endl;
        return;
    }
    
    ir = 0;
    while (fread.getline(buf, sizeof(buf)))
    {
        result = split(buf, " ,\t");
        ROI = cv::Rect(atoi(result[0].c_str()), atoi(result[1].c_str()), atoi(result[2].c_str()), atoi(result[3].c_str()));
        ir++;
    }
    std::cout << ROI << std::endl;
    fread.close();

    //获取Anew
    str = parameterPath + "/map/SHL01/Anew.log";
    result.clear();
    fread.open(str);
    if (!fread.is_open())
    {
        std::cout << "读取文件" + str + "失败" << std::endl;
        return;
    }
    ir = 0;
    
    while (fread.getline(buf, sizeof(buf)))
    {
        result = split(buf, " ,\t");
        Anew.at<float>(ir, 0) = atof(result[0].c_str());
        Anew.at<float>(ir, 1) = atof(result[1].c_str());
        Anew.at<float>(ir, 2) = atof(result[2].c_str());
        ir++;
    }
    std::cout << Anew << std::endl;
    fread.close();
}

cv::Mat MapCalculator::rectify(cv::Mat mat)
{
    
    cv::Mat res;
    cv::remap(mat, res, getCPUMapx(), getCPUMapy(), cv::INTER_LINEAR);
    std::cout <<"res: " << res.size() << std::endl;
    return res;
}


void MapCalculator::initMat()
{
 
    cpu_mapx.create(image_height, image_width, CV_32FC1);
    cpu_mapy.create(image_height, image_width, CV_32FC1);
}


void MapCalculator::calcImageMap( cv::Mat& mapx, cv::Mat& mapy)
{
//{
    // distortion parameters and rectification
    const float k1 = D.at<float>(0, 0);
    const float k2 = D.at<float>(1, 0);
    const float k3 = D.at<float>(2, 0);
    const float p1 = D.at<float>(3, 0);
    const float p2 = D.at<float>(4, 0);
    const cv::Mat R = this->R;

    /* in configuration file, cx and cy is the average of cx1/cx2 and cy1/cy2,
     since the size of image is doubled, we double the new cx and cy */
    const float cx_new = 2 * Anew.at<float>(0, 2);
    const float cy_new = 2 * Anew.at<float>(1, 2);

    int height = mapx.rows;
    int width = mapx.cols;

    // calculate the map, starting from row 1 to end.
    for (int y = 0; y < height; ++y)
    {
        float pos[3] = { 0 };
        float *mapx_row = mapx.ptr<float>(y);
        float *mapy_row = mapy.ptr<float>(y);

        for (int x = 0; x < width; ++x)
        {
            float u = (x - cx_new) / Anew.at<float>(0, 0);
            float v = (y - cy_new) / Anew.at<float>(1, 1);
            pos[0] = R.at<float>(0, 0) * u + R.at<float>(0, 1) * v + R.at<float>(0, 2);
            pos[1] = R.at<float>(1, 0) * u + R.at<float>(1, 1) * v + R.at<float>(1, 2);
            pos[2] = R.at<float>(2, 0) * u + R.at<float>(2, 1) * v + R.at<float>(2, 2);
            u = pos[0] / pos[2];
            v = pos[1] / pos[2];

            float r2 = u * u + v * v;
            float _2uv = 2 * u * v;
            float kr = 1 + r2 * (k1 + r2 * (k2 + r2 * k3));
            float ud = u * kr + p1 * _2uv + p2 * (r2 + 2 * u * u);
            float vd = v * kr + p2 * _2uv + p1 * (r2 + 2 * v * v);
            mapx_row[x] = A.at<float>(0, 0) * ud + A.at<float>(0, 2);
            mapy_row[x] = A.at<float>(1, 1) * vd + A.at<float>(1, 2);
        }
    }
}


std::vector<std::string> MapCalculator::split(const std::string& s, const std::string& seperator)
{
    std::vector<std::string> result;
    typedef std::string::size_type string_size;
    string_size i = 0;

    while (i != s.size()) {
        //找到字符串中首个不等于分隔符的字母；
        int flag = 0;
        while (i != s.size() && flag == 0) {
            flag = 1;
            for (string_size x = 0; x < seperator.size(); ++x)
                if (s[i] == seperator[x]) {
                    ++i;
                    flag = 0;
                    break;
                }
        }

        //找到又一个分隔符，将两个分隔符之间的字符串取出；
        flag = 0;
        string_size j = i;
        while (j != s.size() && flag == 0) {
            for (string_size x = 0; x < seperator.size(); ++x)
                if (s[j] == seperator[x]) {
                    flag = 1;
                    break;
                }
            if (flag == 0)
                ++j;
        }
        if (i != j) {
            result.push_back(s.substr(i, j - i));
            i = j;
        }
    }
    return result;
}
