#include "marker_detection.h"
#include <opencv2/core/eigen.hpp>
#include "optimization_auxilliary.h"

MarkerDetection::MarkerDetection()
{
	memset((char*)&measurement_params, 0, sizeof(measurement_params));
}

MarkerDetection::MarkerDetection(MEASUREMENT_SYSTEM_PARAMS mp)
{
	memcpy((char*)&measurement_params, (char*)&mp, sizeof(mp));

}

bool MarkerDetection::getPoseFromFrames(const bool is_global, std::vector<PATTERN_CONTAINER>& left_marker_decal,
	std::vector<PATTERN_CONTAINER>& right_marker_decal, cv::Rect& left_roi, 
	cv::Rect& right_roi, Eigen::Matrix4d& pose, float& residue)
{
	if (false == checkRepetitivePattern(left_marker_decal, right_marker_decal))
	{
		if (!is_global) {
			setROI(left_roi, measurement_params.other_params.roi_incremental_ratio.unfounded_ratio);
			setROI(right_roi, measurement_params.other_params.roi_incremental_ratio.unfounded_ratio);
		}
		return false;
	}
	Corners leftPoints2d, rightPoints2d;
	std::vector<Eigen::Vector3d> leftPoints3d, rightPoints3d;
	Eigen::Matrix3d A_eig = measurement_params.camera.rectified_camera_params.A_cam;
	double t = measurement_params.camera.rectified_camera_params.b_dis;
	Eigen::Matrix3d R_res = Eigen::Matrix3d::Identity();
	Eigen::Vector3d t_res = Eigen::Vector3d::Zero();
	Eigen::Matrix4d Tcam = Eigen::Matrix4d::Identity();
	double cost = 0.0;
	bool founded = false;

	processOneFrame(1, is_global, left_roi, left_marker_decal,leftPoints2d, leftPoints3d);
	processOneFrame(0, is_global, right_roi, right_marker_decal, rightPoints2d, rightPoints3d);

	// calculate T initial.
	std::vector<Eigen::Matrix3d> rot_inits;
	std::vector<Eigen::Vector3d> trans_inits;

    // use left camera points to solve pnp as the initial value.
	std::vector<Eigen::Vector3d>::const_iterator first3d = leftPoints3d.begin();
	std::vector<Eigen::Vector3d>::const_iterator last3d = leftPoints3d.begin() + 7;
	std::vector<Eigen::Vector3d> p3d_onemk(first3d, last3d);
	Corners::const_iterator first2d = leftPoints2d.begin();
	Corners::const_iterator last2d = leftPoints2d.begin() + 7;
	Corners p2d_onemk(first2d, last2d);

	Eigen::Matrix3d rotm_1, rotm_2;
	Eigen::Vector3d trans_1, trans_2;
	calcAnalyticPosebyIPPE(p2d_onemk, p3d_onemk, rotm_1, trans_1, rotm_2, trans_2);
	rot_inits.push_back(rotm_1);
	trans_inits.push_back(trans_1);
	rot_inits.push_back(rotm_2);
	trans_inits.push_back(trans_2);
	
	// 最终优化
	optInTwoImagesNonePlane(leftPoints2d, leftPoints3d, rightPoints2d, rightPoints3d,
		rot_inits, trans_inits, A_eig, t, R_res, t_res, cost);

	if (cost < 5.0)
	{
		Tcam << R_res(0, 0), R_res(0, 1), R_res(0, 2), t_res(0),
			R_res(1, 0), R_res(1, 1), R_res(1, 2), t_res(1),
			R_res(2, 0), R_res(2, 1), R_res(2, 2), t_res(2),
			0.0, 0.0, 0.0, 1.0;
		founded = true;
	}

	pose = Tcam;
	residue = cost;
	if (founded)
		return true;
	else
		return false;
}

bool MarkerDetection::checkRepetitivePattern(std::vector<PATTERN_CONTAINER>& left_marker_decal,
	std::vector<PATTERN_CONTAINER>& right_marker_decal){
	std::vector<unsigned short> left_id, right_id;
	for (size_t i = 0; i < right_marker_decal.size(); i++)
		right_id.emplace_back(right_marker_decal[i].marker_pixel_id);
	for (int i = left_marker_decal.size() - 1; i > -1; i--) {
		std::vector<unsigned short>::iterator it;
		it = std::find(right_id.begin(), right_id.end(), left_marker_decal[i].marker_pixel_id);
		if (it == right_id.end()) {
			left_marker_decal.erase(left_marker_decal.begin() + i);
		}
	}
	for (size_t i = 0; i < left_marker_decal.size(); i++)
		left_id.emplace_back(left_marker_decal[i].marker_pixel_id);
	for (int i = right_marker_decal.size() - 1; i > -1; i--) {
		std::vector<unsigned short>::iterator it;
		it = std::find(left_id.begin(), left_id.end(), right_marker_decal[i].marker_pixel_id);
		if (it == left_id.end()) {
			right_marker_decal.erase(right_marker_decal.begin() + i);
		}
	}
	if (left_marker_decal.size() > 0 && right_marker_decal.size() > 0)
		return true;
	else
		return false;
}

void MarkerDetection::calcAnalyticPosebyIPPE(const Corners& corners_2D, const std::vector<Eigen::Vector3d>& points_3D,
	Eigen::Matrix3d& matrixTemp1, Eigen::Vector3d& vectTemp1, Eigen::Matrix3d& matrixTemp2, Eigen::Vector3d& vectTemp2){
	std::vector<cv::Point3f> object3DPoints;//存储四个点的世界坐标
	std::vector<cv::Point2f> image2DPoints;//存储四个点的图像坐标
	cv::Mat camera_matrix;//内参数矩阵
	cv::Mat distortion_coefficients;//畸变系数
	Eigen::Matrix3d A_cam = measurement_params.camera.rectified_camera_params.A_cam;
	camera_matrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
	camera_matrix.ptr<double>(0)[0] = A_cam(0, 0);
	camera_matrix.ptr<double>(0)[2] = A_cam(0, 2);
	camera_matrix.ptr<double>(1)[1] = A_cam(1, 1);
	camera_matrix.ptr<double>(1)[2] = A_cam(1, 2);
	camera_matrix.ptr<double>(2)[2] = 1.0f;
	distortion_coefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));

	//设置特征点的世界坐标,图像坐标
	for (int i = 0; i < points_3D.size(); i++)
	{
		//三维坐标的单位是毫米
		object3DPoints.push_back(cv::Point3f(points_3D.at(i)(0), points_3D.at(i)(1), 0));
		image2DPoints.push_back(corners_2D.at(i));
	}

	IPPE::PoseSolver planePoseSolver;
	cv::Mat rvec1, tvec1; //first pose
	cv::Mat rvec2, tvec2; //second pose
	float err1, err2; //RMS reprojection errors of first and second poses

	planePoseSolver.solveGeneric(object3DPoints, image2DPoints, camera_matrix, distortion_coefficients, 
		rvec1, tvec1, err1, rvec2, tvec2, err2);

	/*******************提取旋转矩阵*********************/
	cv::Mat RoteM1, TransM1;
	double rm1[9];
	RoteM1 = cv::Mat(3, 3, CV_64FC1, rm1);
	cv::Rodrigues(rvec1, RoteM1);
	TransM1 = tvec1;
	cv::cv2eigen(RoteM1, matrixTemp1);
	cv::cv2eigen(TransM1, vectTemp1);

	//another
	cv::Mat RoteM2, TransM2;
	double rm2[9];
	RoteM2 = cv::Mat(3, 3, CV_64FC1, rm2);
	cv::Rodrigues(rvec2, RoteM2);
	TransM2 = tvec2;
	cv::cv2eigen(RoteM2, matrixTemp2);
	cv::cv2eigen(TransM2, vectTemp2);
	return;
}

void MarkerDetection::processOneFrame(const bool is_left, const bool is_global, cv::Rect& rect,
	std::vector<PATTERN_CONTAINER>& marker_decal, Corners& Points2d, std::vector<Eigen::Vector3d>& Points3d) {

	Eigen::Matrix3d camera_rotation, camera_intrinsic, A_cam;
	Eigen::Vector4d	camera_distortion;
	PHISICAL_MARKER_POSE marker_used = measurement_params.marker;
	A_cam = measurement_params.camera.rectified_camera_params.A_cam;
	if (is_left) {
		camera_rotation = measurement_params.camera.rectified_camera_params.rectified_left_camera_rotation;
		camera_intrinsic = measurement_params.camera.initial_camera_params.left_camera_intrinsic;
		camera_distortion = measurement_params.camera.initial_camera_params.left_camera_distortion;
	}
	else {
		camera_rotation = measurement_params.camera.rectified_camera_params.rectified_right_camera_rotation;
		camera_intrinsic = measurement_params.camera.initial_camera_params.right_camera_intrinsic;
		camera_distortion = measurement_params.camera.initial_camera_params.right_camera_distortion;
	}
	if (marker_decal.size() > 0){
		for (size_t i = 0; i < marker_decal.size(); i++){
			for (size_t j = 0; j < MAX_MARKER_SIZE; j++){
				marker_decal[i].figure_coordinations[j] = rectifyOneCorner(marker_decal[i].figure_coordinations[j],
					A_cam, camera_rotation, camera_intrinsic, camera_distortion);
			}
		}
	}
	for (size_t i = 0; i < marker_decal.size(); i++) {
		for (size_t j = 0; j < MAX_MARKER_SIZE; j++) {
			if (marker_decal[i].marker_pixel_id == marker_used.marker_decal_id[j]) { 
				for (size_t k = 0; k < MAX_MARKER_SIZE; k++){
					Eigen::Vector4d point_4d;
					point_4d << marker_used.pose[j].position[k].x,
						marker_used.pose[j].position[k].y, 0.0, 1.0;
					Points2d.emplace_back(marker_decal[i].figure_coordinations[k]);
					point_4d = marker_used.matrixALL[j] * point_4d;
					Points3d.emplace_back(point_4d.head(3));
				}
			}
		}
	}
	if (!is_global){
		std::vector<cv::Point2f> rcr_t;
		int minx(6000), miny(4000), maxx(0), maxy(0);
		for (size_t i = 0; i < marker_decal.size(); i++)
		{
			for (size_t j = 0; j < 4; j++)
			{
				if (marker_decal[i].figure_coordinations[j].x < minx)
					minx = marker_decal[i].figure_coordinations[j].x;
				if (marker_decal[i].figure_coordinations[j].x > maxx)
					maxx = marker_decal[i].figure_coordinations[j].x;
				if (marker_decal[i].figure_coordinations[j].y < miny)
					miny = marker_decal[i].figure_coordinations[j].y;
				if (marker_decal[i].figure_coordinations[j].y > maxy)
					maxy = marker_decal[i].figure_coordinations[j].y;
			}
		}
		rect = cv::Rect(minx,miny,maxx-minx,maxy-miny);
		setROI(rect, measurement_params.other_params.roi_incremental_ratio.founded_ratio);
	}
}

Corner MarkerDetection::rectifyOneCorner(const Corner& cs, const Eigen::Matrix3d& A_cam,
	const Eigen::Matrix3d& rect_rot, const Eigen::Matrix3d& cam_intrins, Eigen::Vector4d& cam_distor) {
	Corner cr_res(0, 0);

	PixelType x_d = (cs.x - cam_intrins(0, 2)) / cam_intrins(0, 0);
	PixelType y_d = (cs.y - cam_intrins(1, 2)) / cam_intrins(1, 1);
	auto pixel_ud = rectifySinglePixel(Corner(x_d, y_d), cam_distor);
	Eigen::Vector3d p_ud; p_ud << pixel_ud.x, pixel_ud.y, 1;
	auto pos = rect_rot * p_ud;
	PixelType l_x_ud_uv = pos(0) / pos(2);
	PixelType l_y_ud_uv = pos(1) / pos(2);

	cr_res.x = A_cam(0, 0) * l_x_ud_uv + A_cam(0, 2);
	cr_res.y = A_cam(0, 0) * l_y_ud_uv + A_cam(1, 2);

	return cr_res;
}

Corner MarkerDetection::rectifySinglePixel(const Corner pixel, const Eigen::Vector4d distortion_coeff) {
	// The variable to solve for with its initial value.
	double x_ud[2] = { pixel.x, pixel.y };

	// Build the problem.
	ceres::Problem problem;

	// Set up the only cost function (also known as residual). This uses
	// auto-differentiation to obtain the derivative (jacobian).
	problem.AddResidualBlock(
		new ceres::AutoDiffCostFunction<CostFunctorDistortion, 2, 2>(
			new CostFunctorDistortion(pixel.x, pixel.y, distortion_coeff(0), distortion_coeff(1), distortion_coeff(2), distortion_coeff(3))),
		nullptr,
		x_ud);

	// Run the solver!
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	ceres::Solver::Summary summary;
	Solve(options, &problem, &summary);
	return Corner(x_ud[0], x_ud[1]);
}

void MarkerDetection::setROI(cv::Rect& rect, double ratio)
{
	int width = measurement_params.camera.rectified_camera_params.image_width;
	int height = measurement_params.camera.rectified_camera_params.image_height;
	int min_size = 200;
	if (rect.width == 0 || rect.height == 0) {
		rect = cv::Rect(0, 0, width, height);
		return;
	}		
	if (rect.width < min_size) {
		rect.x = rect.x - (min_size - rect.width) / 2;
		rect.width = min_size;
	}
	if (rect.height < min_size) {
		rect.y = rect.y - (min_size - rect.height) / 2;
		rect.height = min_size;
	}
	int temp_minx = rect.x;
	int temp_miny = rect.y;
	int temp_maxx = temp_minx + rect.width;
	int temp_maxy = temp_miny + rect.height;
	int dt = std::max(rect.width, rect.height) * ratio;
	temp_minx = std::max(temp_minx - dt, 0);
	temp_miny = std::max(temp_miny - dt, 0);
	temp_maxx = std::min(temp_maxx + dt, width - 1);
	temp_maxy = std::min(temp_maxy + dt, height - 1);
	rect = cv::Rect(temp_minx, temp_miny, temp_maxx - temp_minx, temp_maxy - temp_miny);
}

void MarkerDetection::optInTwoImagesNonePlane(const Corners& left_corners_2D, 
	const std::vector<Eigen::Vector3d>& left_points_3D, const Corners& right_corners_2D, 
	const std::vector<Eigen::Vector3d>& right_points_3D, const std::vector<Eigen::Matrix3d>& R_inits, 
	const std::vector<Eigen::Vector3d>& t_inits, const Eigen::Matrix3d& A_cam, 
	const double& t_cam, Eigen::Matrix3d& R_res, Eigen::Vector3d& t_res, double& cost)
{
	auto rotRPY = [](const double phi, const double theta, const double psi) {
		Eigen::Matrix3d rot;
		rot << cos(phi) * cos(theta), -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi), sin(phi)* sin(psi) + cos(phi) * sin(theta) * cos(psi),
			sin(phi)* cos(theta), cos(phi)* cos(psi) + sin(phi) * sin(theta) * sin(psi), -cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi),
			-sin(theta), cos(theta)* sin(psi), cos(theta)* cos(psi);

		return rot;
	};

	double test_rpy[7][3] = { {0,0,0}, {1.5, 0, 0}, {-1.5, 0, 0}, {0, 1.5, 0},
		{0, -1.5, 0}, {0, 0, 1.5}, {0, 0, -1.5} };
	cost = 1e10;
	for (size_t ti = 0; ti < t_inits.size(); ti++) {
		Eigen::Matrix3d R_init = R_inits.at(ti);
		R_init = Eigen::Matrix3d::Identity();
		Eigen::Vector3d t_init = t_inits.at(ti);
		for (size_t ri = 0; ri < 7; ri++) {
			double pose[6] = { 0.0, 0, 0, 0, 0, 0 };
			Eigen::Matrix3d R_test = R_init * rotRPY(test_rpy[ri][0], test_rpy[ri][1], test_rpy[ri][2]);

			ceres::Problem problem;
			auto problem_count = 0;
			for (int i = 0; i < left_corners_2D.size(); ++i) {
				problem.AddResidualBlock(
					new ceres::AutoDiffCostFunction<ReprojectionErrorNonePlane, 2, 6>(
						new ReprojectionErrorNonePlane(R_test, t_init, A_cam, left_corners_2D.at(i), left_points_3D.at(i))),
					nullptr,
					pose);
				++problem_count;
			}
			for (int i = 0; i < right_corners_2D.size(); ++i) {
				Eigen::Vector3d bias(-t_cam, 0, 0);
				problem.AddResidualBlock(
					new ceres::AutoDiffCostFunction<ReprojectionErrorNonePlane, 2, 6>(
						new ReprojectionErrorNonePlane(R_test, t_init + bias, A_cam, right_corners_2D.at(i), right_points_3D.at(i))),
					nullptr,
					pose);
				++problem_count;
			}

			ceres::Solver::Options options;
			options.trust_region_strategy_type = ceres::DOGLEG;// LEVENBERG_MARQUARDT;//
			options.linear_solver_type = ceres::DENSE_QR;
			options.minimizer_progress_to_stdout = false;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);

			double cost_x = sqrt(summary.final_cost / problem_count);
			if (cost_x < cost) {
				cost = cost_x;
				R_res = R_test * rotRPY(pose[0], pose[1], pose[2]);
				t_res = t_init + Eigen::Vector3d(pose[3], pose[4], pose[5]);
			}
		}
	}
}

bool MarkerDetection::uploadConfig(MEASUREMENT_SYSTEM_PARAMS& msp)
{
	msp = measurement_params;
	return true;
}

Corner MarkerDetection::disRectifyOneCorner(const Corner& corner_, const Eigen::Matrix3d& A_cam,
	const Eigen::Matrix3d& camera_intrinsic, const Eigen::Matrix3d& rect_camera, const Eigen::Vector4d& camera_distortion)
{
	Corner corner_res;

	PixelType l_x_ud_uv, l_y_ud_uv;
	l_x_ud_uv = (corner_.x - A_cam(0, 2)) / A_cam(0, 0);
	l_y_ud_uv = (corner_.y - A_cam(1, 2)) / A_cam(0, 0);
	Eigen::Vector3d pos(l_x_ud_uv, l_y_ud_uv, 1);
	auto p_ud = rect_camera.inverse() * pos;
	Corner pixel_ud = Corner(p_ud(0) / p_ud(2), p_ud(1) / p_ud(2));

	double r_xy = (pixel_ud.x * pixel_ud.x + pixel_ud.y * pixel_ud.y);
	auto k1 = camera_distortion(0);
	auto k2 = camera_distortion(1);
	auto p1 = camera_distortion(2);
	auto p2 = camera_distortion(3);
	auto x_d = pixel_ud.x * (1 + (k1)*r_xy + (k2)*r_xy * r_xy) + (2) * (p1)*pixel_ud.x * pixel_ud.y + (p2) * (r_xy + (2) * pixel_ud.x * pixel_ud.x);
	auto y_d = pixel_ud.y * ((1) + (k1)*r_xy + (k2)*r_xy * r_xy) + (2) * (p2)*pixel_ud.x * pixel_ud.y + (p1) * (r_xy + (2) * pixel_ud.y * pixel_ud.y);

	corner_res.x = camera_intrinsic(0, 0) * x_d + camera_intrinsic(0, 2);
	corner_res.y = camera_intrinsic(1, 1) * y_d + camera_intrinsic(1, 2);

	return corner_res;
}