#include "line2Dup.h"
#include <memory>
#include <iostream>
#include <assert.h>
#include <chrono>
#include <opencv2/dnn.hpp>
#include <opencv2\imgproc\types_c.h>
//using namespace std;
//using namespace cv;

//class Timer
//{
//public:
//    Timer() : beg_(clock_::now()) {}
//    void reset() { beg_ = clock_::now(); }
//    double elapsed() const {
//        return std::chrono::duration_cast<second_>
//            (clock_::now() - beg_).count(); }
//    void out(std::string message = ""){
//        double t = elapsed();
//        std::cout << message << "  elasped time:" << t << "s" << std::endl;
//        reset();
//    }
//private:
//    typedef std::chrono::high_resolution_clock clock_;
//    typedef std::chrono::duration<double, std::ratio<1> > second_;
//    std::chrono::time_point<clock_> beg_;
//};

static std::string prefix = "./test_img/dhzzzq/";

void circle_gen(){
	cv::Mat bg = cv::Mat(800, 800, CV_8UC3, { 0, 0, 0 });
    cv::circle(bg, {400, 400}, 200, {255,255,255}, -1);
    cv::imshow("test", bg);
	cv::waitKey(0);
}

int angle_test(std::string mode = "test"){
    line2Dup::Detector detector(32, {2, 4});  // 32 features are not precise enough; {4,8}

	//    mode = "none";
    if(mode == "train"){

		std::vector<std::string> class_ids{ "up", "down" };

		for (std::string& class_id : class_ids){
			cv::Mat img = cv::imread(prefix  + class_id + ".png");
			if (img.empty())
				std::cerr << "image path wrong!" << std::endl;

			// if type == needleHolder or shear, then resize the img.
			cv::resize(img, img, cv::Size(), 2.0, 2.0);

			cv::Mat mask = cv::Mat(img.size(), CV_8UC1, { 255 });

			// padding to avoid rotating out
			int padding = 100;
			cv::Mat padded_img = cv::Mat(img.rows + 2*padding, img.cols + 2*padding, img.type(), cv::Scalar::all(0));
			img.copyTo(padded_img(cv::Rect(padding, padding, img.cols, img.rows)));
			cv::Mat padded_mask = cv::Mat(mask.rows + 2*padding, mask.cols + 2*padding, mask.type(), cv::Scalar::all(0));
			mask.copyTo(padded_mask(cv::Rect(padding, padding, img.cols, img.rows)));

			shape_based_matching::shapeInfo shapes(padded_img, padded_mask);
			shapes.angle_range = {0, 360};
			shapes.angle_step = 1;
			shapes.produce_infos();
			//std::vector<shape_based_matching::shapeInfo::shape_and_info> infos_have_templ;  // shape info is not necessary
			for(auto& info: shapes.infos){
				std::cout << "\ninfo.angle: " << info.angle << std::endl;
				int templ_id = detector.addTemplate(info.src, class_id, info.mask);
				std::cout << "templ_id: " << templ_id << std::endl;
				//if(templ_id != -1){  
				//    infos_have_templ.push_back(info);
				//}
			}
		}
		detector.writeClasses(prefix +"%s_templ.yaml");
        //shapes.save_infos(infos_have_templ, shapes.src, shapes.mask, prefix + "case4/test_info.yaml");
        std::cout << "train end" << std::endl;
		return 0;

    }else if(mode=="test"){
		std::vector<std::string> class_ids{ "up", "down" };
		detector.readClasses(class_ids, prefix +"%s_templ.yaml");

		//cv::Mat test_img = cv::imread(prefix + "dualGrapper/test5.png");

		cv::Mat test_img = cv::imread("F:/code_git/pythonScripts/rotate_figure/pic/DHZZZQ.png");
		cv::resize(test_img, test_img, cv::Size(), 2.0, 2.0);
       /* int padding = 250;
        cv::Mat padded_img = cv::Mat(test_img.rows + 2*padding,
                                     test_img.cols + 2*padding, test_img.type(), cv::Scalar::all(0));
        test_img.copyTo(padded_img(Rect(padding, padding, test_img.cols, test_img.rows)));*/
		
        int stride = 16;
        int n = test_img.rows/stride;  // padded_img
        int m = test_img.cols/stride;  // padded_img
		cv::Rect roi(0, 0, stride*m, stride*n);
		cv::Mat img = test_img(roi).clone();  // padded_img
        assert(img.isContinuous());

        //cvtColor(img, img, CV_BGR2GRAY);

        std::cout << "test img size: " << img.rows * img.cols << std::endl;

        //Timer timer;
        auto matches = detector.match(img, 36, class_ids);
        //timer.out();

        if(img.channels() == 1) cvtColor(img, img, CV_GRAY2BGR);

        std::cout << "matches.size(): " << matches.size() << std::endl;
        size_t top5 = 400;
        if(top5>matches.size()) top5=matches.size();
		int rotate_angle = 360;
		std::vector<int> flags(class_ids.size(), 0);
        for(size_t i=0;i<top5;i++){
			//std::cout << i << std::endl;
			auto match = matches[i];
			
			// we want to find 2 top match
			std::vector<std::string>::iterator it = find(class_ids.begin(), class_ids.end(), match.class_id);
			if (flags[distance(class_ids.begin(), it)] == 0){
				flags[distance(class_ids.begin(), it)] = 1;
				auto templ = detector.getTemplates(match.class_id,match.template_id);

				float train_img_half_width = templ[0].width / 2.0f ;
				float train_img_half_heigth = templ[0].height / 2.0f;

				// center x,y of train_img in test img
				float x = match.x + train_img_half_width; 
				float y = match.y + train_img_half_heigth;

				cv::Vec3b randColor;
				randColor[0] = rand() % 155 + 100;
				randColor[1] = rand() % 155 + 100;
				randColor[2] = rand() % 155 + 100;
				for (int i = 0; i < templ[0].features.size(); i++){
					auto feat=templ[0].features[i];
					cv::circle(img, { feat.x + match.x, feat.y + match.y }, 3, randColor, -1);
				}

				rotate_angle = match.template_id;
				cv::putText(img, std::to_string(int(round(match.similarity))) + "; angle: " + std::to_string(rotate_angle),
					cv::Point(match.x + train_img_half_width - 40, match.y - 3), cv::FONT_HERSHEY_PLAIN, 1, randColor);
				
				cv::RotatedRect rotatedRectangle({ x, y }, { 2 * train_img_half_width, 2 * train_img_half_heigth }, 0);
				std::cout << rotate_angle << std::endl;

				cv::Point2f vertices[4];
				rotatedRectangle.points(vertices);
				for (int i = 0; i<4; i++){
					int next = (i + 1 == 4) ? 0 : (i + 1);
					line(img, vertices[i], vertices[next], randColor, 2);
				}

				std::cout << "\nmatch.template_id: " << match.template_id << std::endl;
				std::cout << "match.similarity: " << match.similarity << std::endl;
			}
			if (find(flags.begin(), flags.end(), 0) == flags.end())
				break;
        }

		cv::imshow("result", img);
		cv::waitKey(0);
		//cv::imwrite("result.jpg", img);

        std::cout << "test end" << std::endl;
		return rotate_angle;
    }
}

int main(){
    int rotate_angle = angle_test("test");
    return 0;
}
