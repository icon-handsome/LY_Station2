#include "TR_Mark_Track.h"
#include <algorithm>
#include <cstring>
#include <cmath>
#include <cfloat>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>

// Set LB_ENABLE_CONSOLE_LOG=1 at compile time to restore verbose LB output.
#ifndef LB_ENABLE_CONSOLE_LOG
#define LB_ENABLE_CONSOLE_LOG 0
#endif
#if LB_ENABLE_CONSOLE_LOG
#define LB_COUT std::cout
#else
struct LbNullBuf : public std::streambuf
{
	int overflow(int c) override { return c; }
};

inline std::ostream& lbNullOut()
{
	static LbNullBuf buf;
	static std::ostream out(&buf);
	return out;
}
#define LB_COUT lbNullOut()
#endif

namespace
{
	inline float ClampCos(float value)
	{
		if (value < -1.0f)
		{
			return -1.0f;
		}
		if (value > 1.0f)
		{
			return 1.0f;
		}
		return value;
	}

	inline float DegreesToRadians(float degrees)
	{
		return degrees * 3.14159265358979323846f / 180.0f;
	}

	double TransformDistance(const cv::Mat& Rt,
		                     const cv::Point3f& src,
		                     const cv::Point3f& dst)
	{
		cv::Mat pt_homo = (cv::Mat_<double>(4, 1) << src.x, src.y, src.z, 1.0);
		cv::Mat trans_p = Rt * pt_homo;
		double dx = trans_p.at<double>(0, 0) - dst.x;
		double dy = trans_p.at<double>(1, 0) - dst.y;
		double dz = trans_p.at<double>(2, 0) - dst.z;
		return std::sqrt(dx * dx + dy * dy + dz * dz);
	}

	void SelectPoseInliers(const cv::Mat& Rt,
		                   const std::vector<cv::Point3f>& template_points,
		                   const std::vector<cv::Point3f>& frame_points,
		                   double dist_thresh,
		                   std::vector<cv::Point3f>& inlier_template_points,
		                   std::vector<cv::Point3f>& inlier_frame_points)
	{
		inlier_template_points.clear();
		inlier_frame_points.clear();
		inlier_template_points.reserve(template_points.size());
		inlier_frame_points.reserve(frame_points.size());

		for (size_t i = 0; i < frame_points.size(); ++i)
		{
			double dist = TransformDistance(Rt, template_points[i], frame_points[i]);
			if (dist < dist_thresh)
			{
				inlier_template_points.push_back(template_points[i]);
				inlier_frame_points.push_back(frame_points[i]);
			}
		}
	}
}

/**************************************************************************************
*��  �ܣ���ǵ�ͼ��Ԥ�����������
*��  ����
*       img_in                      I         ����ĸ߷ֱ���ͼ
*       results                     O         ����ı�ǵ�Բ������������
*����ֵ��״̬��
*��  ע���ú���������չ��ȫ����ֲ������л��ĺ���
**************************************************************************************/
bool MarkPointDetector::ProcessFrame(const cv::Mat& img_in, 
		                             std::vector<cv::Point2f>& results) 
{
	bool status = GlobalSearch(img_in, results);
	return status;
}


/**************************************************************************************
*��  �ܣ�ͼ��������ɴֵ��� (���ڳ�ʼ������ٶ�ʧ)
*��  ����
*       img_in                      I         ����ĸ߷ֱ���ͼ
*       results                     O         ����ı�ǵ�Բ������������
*����ֵ��״̬��
*��  ע��
**************************************************************************************/
bool MarkPointDetector::GlobalSearch(const cv::Mat&            img_in,
	                                 std::vector<cv::Point2f>  &results)
{
	results.clear();
	std::vector<cv::Point2f> circle_centers;
	circle_centers.reserve(300);
	std::vector<float> mark_area;
	mark_area.resize(1000);

	// ͼ��ƽ��
	cv::Mat blurred;
	cv::GaussianBlur(img_in, blurred, cv::Size(5, 5), 1.5);

	// 1. Downsample according to detector configuration.
	cv::Mat small_img = blurred.clone();
	int level_cnt = config.pyramid_levels;
	if (level_cnt < 0)
	{
		level_cnt = 0;
	}
	for (int lv = 0; lv < level_cnt; ++lv)
	{
		cv::pyrDown(small_img, small_img);
	}
	//cv::imwrite("0 small-IMG.jpg", small_img, { cv::IMWRITE_JPEG_QUALITY, 90 });

	// 2. ����ȡ (�򵥵���ֵ + ��������)
	cv::Mat binary;

	int channels = small_img.channels();

	if (channels == 1)
	{
		LB_COUT << "ͼ���Ѿ��ǻҶ�ͼ����ͨ������" << std::endl;
	}
	else if (channels == 3)
	{
		LB_COUT << "ͼ��Ϊ3ͨ����ɫͼ������ת��Ϊ�Ҷ�ͼ..." << std::endl;
		cv::cvtColor(small_img, small_img, cv::COLOR_BGR2GRAY);
	}
	else if (channels == 4)
	{
		LB_COUT << "ͼ��Ϊ4ͨ����ɫͼ����Alphaͨ����������ת��Ϊ�Ҷ�ͼ..." << std::endl;
		cv::cvtColor(small_img, small_img, cv::COLOR_BGRA2GRAY);
	}
	else
	{
		LB_COUT << "δ֪��ͨ����: " << channels << "�����Խ��б�׼ת��..." << std::endl;
		return 1010;
	}

	//cv::threshold(small_img, binary, 30, 200, cv::THRESH_BINARY);
	cv::adaptiveThreshold(small_img, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 21, 10); // 21
	//cv::threshold(small_img, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	////cv::bitwise_not(binary, binary);
	cv::imwrite("contours_binary.jpg", binary, { cv::IMWRITE_JPEG_QUALITY, 90 });

	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point>> contours_t;
	std::vector<std::vector<cv::Point>> contours;

	////cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	// cv::CHAIN_APPROX_NONE  cv::CHAIN_APPROX_SIMPLE
	//cv::findContours(binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);    // RETR_LIST ����ȡ�������������ٺ����ڲ��׶�  cv::CHAIN_APPROX_SIMPLE ����ѹ�������� 
	cv::findContours(binary, contours_t, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	// TODO: ����Ƿ����ͬ��Բ������
	// hierarchy[i] �ĽṹΪ: [ͬ���һ��, ͬ��ǰһ��, ��һ��������, ������]
	// hierarchy[i][3] ��ʾ��������������
	// ��� hierarchy[i][3] >= 0��˵����ǰ������һ���ڲ��������׶���
	contours.reserve(contours_t.size());
	for (size_t i = 0; i < contours_t.size(); ++i)
	{
		// hierarchy[i][2] -> First_Child (��һ������������)
		// hierarchy[i][3] -> Parent (����������)

		// ��� 1���Ƕ�����������ڲ㣨û����������
		// ֻҪ��ǰ����û������������ hierarchy[i][2] == -1�������ͱ�Ȼ�ǡ����ڲ㡱��������������Ƕ�׵ĵ���������
		bool is_innermost_or_single = (hierarchy[i][2] == -1);

		if (is_innermost_or_single)
		{
			contours.push_back(contours_t[i]);
		}
	}

	// �����������������
	cv::Mat color_result;
	if (1)
	{
		std::vector<std::vector<cv::Point>> scaled_contours = contours; // ��Ҫ�޸�ԭʼ��contours
		for (size_t i = 0; i < scaled_contours.size(); i++)
		{
			for (size_t j = 0; j < scaled_contours[i].size(); j++)
			{
				// ��Ϊ���� 3 �� pyrDown����������Ҫ���� 2^3 = 8
				scaled_contours[i][j].x *= pow(2, level_cnt);
				scaled_contours[i][j].y *= pow(2, level_cnt);
			}
		}
		// 2. ׼����ɫ����
		if (img_in.channels() == 1)
			cv::cvtColor(img_in, color_result, cv::COLOR_GRAY2BGR);
		else
			color_result = img_in.clone();
		// 3. ��������
		// ����������, ��������, ����ȫ��(-1), ��ɫ(��ɫ), ��������(10)
		cv::drawContours(color_result, scaled_contours, -1, cv::Scalar(0, 255, 0), 2);
		//// 4. (��ѡ) ͬʱ����֮ǰ��������������� circle_centers
		//for (const auto& pt : circle_centers)
		//{
		//	cv::circle(color_result, pt, 25, cv::Scalar(0, 0, 255), -1); // ����ɫʵ��ԭ��
		//}
		for (int i = 0; i < scaled_contours.size(); i++)
		{
			std::string text = std::to_string(i);
			cv::putText(color_result, text, scaled_contours[i][0], cv::FONT_HERSHEY_SIMPLEX,
				0.4, cv::Scalar(0, 0, 255), 1);
		}
		// 5. ������
		// 25MP ͼ���鱣��Ϊ JPG �Խ�ʡ�ռ�
		cv::imwrite("contours_result.jpg", color_result, { cv::IMWRITE_JPEG_QUALITY, 90 });
	}

	tracked_points.clear();
	int ii = 0;
	double perim_t = config.perimeter_radius_px / pow(2, level_cnt) * 6.28;     // �뾶5������
	for (auto& cnt : contours)
	{
		if (ii == 10)
		{
			int aaa = 0;
		}
		ii++;

		double area = cv::contourArea(cnt);
		if (area > (config.min_area / pow(4, level_cnt)) && area < (config.max_area / pow(4, level_cnt)))
		{
			// Բ�ȹ��� (Circularity)
			// �ж��Ƿ�Բ����ֹ��ȡ��ǽ�졢�ſ������
			double perimeter = cv::arcLength(cnt, true);
			if (perimeter < perim_t)            // �뾶С��5���صĶ��޳���5/64 * 2 * 3.14 = 0.49
			{
				continue;
			}
			double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
			if (circularity > config.min_circularity) // Խ�ӽ�1ԽԲ����ҵ��ǵ�ͨ�� > 0.8
			{
				// ��С�������ֵ
				cv::Moments mu = cv::moments(cnt);
				cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);

				// ��ԭ���߷ֱ�������
				cv::Point2f high_res_pos(center.x * pow(2, level_cnt), center.y * pow(2, level_cnt));
				// float radius = sqrtf(area / 3.14159265); // 1/3.14159265 * 0.5
				float radius = perimeter * 0.159155 * pow(2, level_cnt); // 1/3.14159265 * 0.5

				if (0)                  // �����˲�
				{
					int x_f = 0;
					int y_f = 0;
					int x_c = 0;
					int y_c = 0;
					if ((int)high_res_pos.x > (img_in.cols - 5) || (int)high_res_pos.y > (img_in.rows - 5) ||
						(int)high_res_pos.x < 5 || (int)high_res_pos.y < 5)
					{
						continue;
					}

					x_f = floorf(high_res_pos.x);
					x_c = ceilf(high_res_pos.x);
					y_f = floorf(high_res_pos.y);
					y_c = ceilf(high_res_pos.y);

					int intensity_c = config.intensity_threshold;
					if ((int)img_in.at<uchar>(y_f, x_f) < intensity_c ||
						(int)img_in.at<uchar>(y_c, x_f) < intensity_c ||
						(int)img_in.at<uchar>(y_f, x_c) < intensity_c ||
						(int)img_in.at<uchar>(y_c, x_c) < intensity_c)
					{
						continue;
					}

					//// ��Ե������ֵ�˲�����ɫ��
					//int intensity_r = 30;
					//int round_x     = x_c;
					//int round_y     = y_c;
					//int radius_int  = floorf(radius) + 3;
					//if ((int)img_in.at<uchar>(round_y, round_x + radius_int) > intensity_r ||
					//	(int)img_in.at<uchar>(round_y, round_x - radius_int) > intensity_r ||
					//	(int)img_in.at<uchar>(round_y + radius_int, round_x) > intensity_r ||
					//	(int)img_in.at<uchar>(round_y - radius_int, round_x) > intensity_r)
					//{
					//	continue;
					//}

				}

				// ��һ����ԭͼ�н��������ؾ���
				cv::Point2f refined_pos;
				//cv::Mat I1 = (cv::Mat_<double>(3, 3) << 5.078966884152220e+03, 1.144606033008170, 2.731696208984134e+03,
				//                                       0.0,                   5.076838475599669e+03,  1.832530560892915e+03,
				//							  0.0,                   0.0,                    1.0);
				//cv::Mat D1 = (cv::Mat_<double>(1, 5) << -0.061814641591874,       // k1
				//                                       0.134149054707469,        // k2
				//							  -1.780078171601695e-04,   // p1
				//							  -5.409994817555799e-04,   // p2
				//							  -0.101638840770598);      // k3


				//refined_pos = RefineCenter(blurred, high_res_pos, radius,I1,D1);

				refined_pos = RefineCenter(blurred, high_res_pos, radius);

				//if (cnt.size() < 5)
				//{
				//	//refined_pos = RefineSubpixel(blurred, high_res_pos);   // img_in
				//	continue;
				//}
				//else
				//{
				//	cv::RotatedRect ellipse = cv::fitEllipse(cnt);
				//	refined_pos = ellipse.center * 8;
				//}


				//TrackedPoint tp;
				//tp.pos2d = refined_pos;
				//tp.is_lost = false;
				//if (!tp.is_lost)
				//{
				//	tp.consecutive_frames++;
				//}
				//tracked_points.push_back(tp);
				circle_centers.push_back(refined_pos); // refined_pos
			}
		}
	}

	// 4. ������������������� circle_centers
	if (1)
	{
		if (img_in.channels() == 1)
			cv::cvtColor(img_in, color_result, cv::COLOR_GRAY2BGR);
		else
			color_result = img_in.clone();
		for (const auto& pt : circle_centers)
		{
			cv::circle(color_result, pt, 1, cv::Scalar(0, 0, 255), -1); // ����ɫʵ��ԭ��
		}
		// 5. ������
		// 25MP ͼ���鱣��Ϊ JPG �Խ�ʡ�ռ�
		cv::imwrite("0centers_result.jpg", color_result, { cv::IMWRITE_JPEG_QUALITY, 90 });
	}

	is_initialized = !circle_centers.empty();

	// ע��Ҫ����Ⱥ��ͳ���˲�
	//std::vector<cv::Point2f> results_2;
	//filterOutliers(circle_centers, results, 2.0);
	//filterOutliers(results, results_2, 2.0);
	//results.clear();
	//filterOutliers(results_2, results, 2.0);

	// TODO: �Ż���ʱ,�Ż��ж�����
	filterOutlies_Debscan(circle_centers,
		results,
		AppConfig::Instance().limits.debscan_filter_dist_max, // cluster radius
		config.debscan_min_pts);

	if (results.size() < 4)
	{
		std::vector<cv::Point2f> results_2;
		filterOutliers(circle_centers, results, 2.0);
		filterOutliers(results, results_2, 1.5);
		results.clear();
		filterOutliers(results_2, results, 1.0);
	}

	//// �Դֶ�λ������о�ϸ��ǵ�Բ����ȡ
	//for (int ii = 0; ii < results.size(); ii++)
	//{
	//	cv::Point2f refined_pos;
	//	RefineCenter(blurred, refined_pos, )

	//}


	//// 6. ������
	//LB_COUT << "circle_centers:              " << circle_centers.size() << std::endl;
	//for (size_t i = 0; i < circle_centers.size(); ++i)
	//{
	//	LB_COUT << circle_centers[i].x << "," << circle_centers[i].y << "," << 0.0f<< std::endl;
	//}
	LB_COUT << "results:                     " << results.size() << std::endl;
	for (size_t i = 0; i < results.size(); ++i)
	{
		LB_COUT << results[i].x << "," << results[i].y << "," << 0.0f << std::endl;
	}

	return is_initialized;
}


///**************************************************************************************
//* ���ܣ���ǵ�ȫ��������ͨ· A Ϊ�����(��ֵ��+�������)��DoG ͨ· B ֻ���� A δ��⵽�ĵ㡣
//* �Բ����ִ�и��ϸ����״У�飬�����㾫�ܲ���Ҫ��
//**************************************************************************************/
//bool MarkPointDetector::GlobalSearch(const cv::Mat& img_in,
//                                     std::vector<cv::Point2f>& results)
//{
//    results.clear();
//    std::vector<cv::Point2f> circle_centers;
//    circle_centers.reserve(300);
//    if (img_in.empty()) return false;
//
//    cv::Mat gray_img;
//    if (img_in.channels() == 1) gray_img = img_in;
//    else if (img_in.channels() == 3) cv::cvtColor(img_in, gray_img, cv::COLOR_BGR2GRAY);
//    else if (img_in.channels() == 4) cv::cvtColor(img_in, gray_img, cv::COLOR_BGRA2GRAY);
//    else return false;
//
//    cv::Mat blurred;
//    cv::GaussianBlur(gray_img, blurred, cv::Size(5, 5), 1.5);
//    const int level_cnt = std::max(0, config.pyramid_levels);
//    const float scale_pos = static_cast<float>(1 << level_cnt);
//    const float scale_area = scale_pos * scale_pos;
//    cv::Mat small_img = blurred.clone();
//    for (int lv = 0; lv < level_cnt; ++lv) cv::pyrDown(small_img, small_img);
//
//    const float A_MIN_AXIS_RATIO = static_cast<float>(config.min_circularity);
//    // DoG ͨ·����������͸����ԲͶӰ�����ų����������ĸ��š�
//    const float B_MIN_AXIS_RATIO = 0.75f;  // �������
//    const float MIN_MINOR_AXIS_PX = 6.0f;  // ��С��������
//
//    // ͨ· A������Ӧ��ֵ����������ϡ�
//    cv::Mat binary;
//    cv::adaptiveThreshold(small_img, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
//        cv::THRESH_BINARY, 31, -3);
//    cv::imwrite("contours_binary.jpg", binary, { cv::IMWRITE_JPEG_QUALITY, 90 });
//    std::vector<std::vector<cv::Point>> contours;
//    cv::findContours(binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
//
//    // ������������ͼ�ͱ����ʾ��
//    if (1) {
//        cv::Mat contour_debug;
//        if (img_in.channels() == 1) cv::cvtColor(img_in, contour_debug, cv::COLOR_GRAY2BGR);
//        else contour_debug = img_in.clone();
//        for (size_t i = 0; i < contours.size(); ++i) {
//            std::vector<cv::Point> scaled = contours[i];
//            for (size_t j = 0; j < scaled.size(); ++j) {
//                scaled[j].x = cvRound(scaled[j].x * scale_pos);
//                scaled[j].y = cvRound(scaled[j].y * scale_pos);
//            }
//            cv::drawContours(contour_debug, std::vector<std::vector<cv::Point>>(1, scaled), -1,
//                cv::Scalar(0, 255, 0), 2);
//            if (!scaled.empty()) cv::putText(contour_debug, std::to_string(i), scaled[0],
//                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
//        }
//        cv::imwrite("contours_result.jpg", contour_debug, { cv::IMWRITE_JPEG_QUALITY, 90 });
//    }
//
//    tracked_points.clear();
//    const double min_area_scaled = config.min_area / scale_area;
//    const double max_area_scaled = config.max_area / scale_area;
//    const double perim_t = (config.perimeter_radius_px / scale_pos) * 5.5;
//    for (const auto& cnt : contours) {
//        if (cnt.size() < 6) continue;
//        const double area = cv::contourArea(cnt);
//        if (area <= min_area_scaled || area >= max_area_scaled) continue;
//        if (cv::arcLength(cnt, true) < perim_t) continue;
//        const cv::RotatedRect ell = cv::fitEllipse(cnt);
//        const float minor_axis = std::min(ell.size.width, ell.size.height);
//        const float major_axis = std::max(ell.size.width, ell.size.height);
//        if (major_axis <= 1e-4f || minor_axis / major_axis < A_MIN_AXIS_RATIO) continue;
//        if (minor_axis * scale_pos < MIN_MINOR_AXIS_PX) continue;
//        const cv::Point2f pos = ell.center * scale_pos;
//        const float radius = major_axis * 0.5f * scale_pos;
//        const int margin = cvRound(radius) + 2;
//        if (pos.x < margin || pos.x >= gray_img.cols - margin ||
//            pos.y < margin || pos.y >= gray_img.rows - margin) continue;
//        const int ix = cvRound(pos.x), iy = cvRound(pos.y);
//        if (gray_img.at<uchar>(iy, ix) < config.intensity_threshold) continue;
//        circle_centers.push_back(RefineCenter(gray_img, pos, radius));
//    }
//    const size_t path_a_count = circle_centers.size();
//
//    // ͨ· B��ÿִֻ֡��һ����ͨ�������һ�η�ֵͳ�ƣ���ѡ�׶ν�������ʱ������
//    const float expected_r_small = static_cast<float>(config.perimeter_radius_px) / scale_pos;
//    if (expected_r_small >= 1.5f) 
//	{
//        cv::Mat g1, g2, dog, dilated;
//        cv::GaussianBlur(small_img, g1, cv::Size(0, 0), expected_r_small * 0.6f);
//        cv::GaussianBlur(small_img, g2, cv::Size(0, 0), expected_r_small * 1.3f);
//        cv::subtract(g1, g2, dog);
//        cv::dilate(dog, dilated, cv::Mat());
//        double dog_max = 0.0;
//        cv::minMaxLoc(dog, nullptr, &dog_max);
//        const float dog_thresh = std::max(4.0f, static_cast<float>(dog_max * 0.20f));
//        const int border = std::max(2, cvRound(expected_r_small * 1.5f));
//
//        // �û����أ���Ϊ false ʱ�ر���ͨ��ճ��У�飬�Խ��͵�֡������ʱ��true
//		const bool ENABLE_DOG_CONNECTED_COMPONENT_CHECK = false;
//        if (ENABLE_DOG_CONNECTED_COMPONENT_CHECK) 
//		{
//            cv::Mat bright_binary, labels, stats, centroids;
//            cv::threshold(small_img, bright_binary, config.intensity_threshold, 255, cv::THRESH_BINARY);
//            const int label_count = cv::connectedComponentsWithStats(bright_binary, labels, stats, centroids, 8, CV_32S);
//            std::vector<std::vector<cv::Point>> peaks_in_label(label_count);
//            const float peak_merge = std::max(3.0f, expected_r_small * 2.0f);
//            const float peak_merge_sq = peak_merge * peak_merge;
//		    
//            // ͳ��ÿ��������ͨ���еĶ��� DoG ��ֵ������弴��������ճ����
//            for (int y = border; y < dog.rows - border; ++y) 
//		    {
//                const uchar* dog_row = dog.ptr<uchar>(y);
//                const uchar* dil_row = dilated.ptr<uchar>(y);
//                for (int x = border; x < dog.cols - border; ++x) 
//		    	{
//                    if (dog_row[x] < dog_thresh || dog_row[x] != dil_row[x]) continue;
//                    const int label = labels.at<int>(y, x);
//                    if (label <= 0 || label >= label_count) continue;
//                    bool same_peak = false;
//                    for (const auto& peak : peaks_in_label[label]) 
//		    		{
//                        const float dx = static_cast<float>(x - peak.x);
//                        const float dy = static_cast<float>(y - peak.y);
//                        if (dx * dx + dy * dy < peak_merge_sq) 
//		    			{ 
//		    				same_peak = true; 
//		    				break; 
//		    			}
//                    }
//                    if (!same_peak) 
//		    			peaks_in_label[label].push_back(cv::Point(x, y));
//                }
//            }
//		    
//            const float exclusion_radius = static_cast<float>(config.duplicate_exclusion_radius_px);
//            const float exclusion_sq = exclusion_radius * exclusion_radius;
//            for (int y = border; y < dog.rows - border; ++y) 
//		    {
//                const uchar* dog_row = dog.ptr<uchar>(y);
//                const uchar* dil_row = dilated.ptr<uchar>(y);
//                for (int x = border; x < dog.cols - border; ++x)
//		    	{
//                    if (dog_row[x] < dog_thresh || dog_row[x] != dil_row[x]) 
//		    			continue;
//                    const int label = labels.at<int>(y, x);
//                    if (label <= 0 || label >= label_count) 
//		    			continue;
//                    // �����ͨ��Ϊճ����ѡ���ܾ������ģ���ֹ����������֮���α���ġ�
//                    if (peaks_in_label[label].size() != 1) 
//		    			continue;
//		    
//                    const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
//                    const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
//                    const int minor_axis = std::min(width, height);
//                    const int major_axis = std::max(width, height);
//                    // ��С��������ѡ�����Ĳ��ȶ��������뾫�ܲ�����
//                    if (minor_axis * scale_pos < MIN_MINOR_AXIS_PX || major_axis <= 0)
//		    			continue;
//                    if (static_cast<float>(minor_axis) / major_axis < B_MIN_AXIS_RATIO)
//		    			continue;
//		    
//                    const cv::Point2f dog_pos(x * scale_pos, y * scale_pos);
//                    bool duplicate = false;
//                    for (const auto& pt : circle_centers)
//		    		{
//                        const cv::Point2f d = dog_pos - pt;
//                        if (d.dot(d) < exclusion_sq)
//		    			{
//		    				duplicate = true; 
//		    				break;
//		    			}
//                    }
//                    if (duplicate) 
//		    			continue;
//                    const int ix = cvRound(dog_pos.x), iy = cvRound(dog_pos.y);
//                    if (ix < 5 || ix >= gray_img.cols - 5 || iy < 5 || iy >= gray_img.rows - 5) 
//		    			continue;
//                    if (gray_img.at<uchar>(iy, ix) < config.intensity_threshold) 
//		    			continue;
//		    
//                    const cv::Point2f component_center(
//                        static_cast<float>(centroids.at<double>(label, 0)) * scale_pos,
//                        static_cast<float>(centroids.at<double>(label, 1)) * scale_pos);
//                    const float radius = std::max(expected_r_small * scale_pos,
//                        0.5f * major_axis * scale_pos);
//                    circle_centers.push_back(RefineCenter(gray_img, component_center, radius));
//                }
//            }
//        }
//        else 
//		{
//            // ����ģʽ��������ͨ�򡢶�塢�ߴ����ȼ�飬ֻ���� DoG ��ֵ�� A/B ȥ�ء�
//            const float exclusion_radius = static_cast<float>(config.duplicate_exclusion_radius_px);
//            const float exclusion_sq = exclusion_radius * exclusion_radius;
//            for (int y = border; y < dog.rows - border; ++y) 
//			{
//                const uchar* dog_row = dog.ptr<uchar>(y);
//                const uchar* dil_row = dilated.ptr<uchar>(y);
//                for (int x = border; x < dog.cols - border; ++x) 
//				{
//                    if (dog_row[x] < dog_thresh || dog_row[x] != dil_row[x]) 
//						continue;
//                    const cv::Point2f dog_pos(x * scale_pos, y * scale_pos);
//                    bool duplicate = false;
//                    for (const auto& pt : circle_centers)
//					{
//                        const cv::Point2f d = dog_pos - pt;
//                        if (d.dot(d) < exclusion_sq) 
//						{ 
//							duplicate = true;
//							break; 
//						}
//                    }
//					if (duplicate)
//					{
//						continue;
//					}
//
//                    const int ix = cvRound(dog_pos.x), iy = cvRound(dog_pos.y);
//                    if (ix < 5 || ix >= gray_img.cols - 5 || iy < 5 || iy >= gray_img.rows - 5) 
//						continue;
//                    if (gray_img.at<uchar>(iy, ix) < config.intensity_threshold) 
//						continue;
//                    circle_centers.push_back(RefineCenter(gray_img, dog_pos, expected_r_small * scale_pos));
//                }
//            }
//        }
//    }
//
//    // ���� DOG ����Ϊ���ף������Ƚ���� A �㣬�����Բ�ͬ��Դ������ȡƽ����
//    std::vector<cv::Point2f> nms_centers;
//    std::vector<int> nms_sources;
//    const float nms_radius = static_cast<float>(config.duplicate_exclusion_radius_px);
//    const float nms_sq = nms_radius * nms_radius;
//    for (size_t i = 0; i < circle_centers.size(); ++i) 
//	{
//        bool duplicate = false;
//        for (const auto& kept : nms_centers) 
//		{
//            const cv::Point2f d = circle_centers[i] - kept;
//            if (d.dot(d) < nms_sq) 
//			{ 
//				duplicate = true;
//				break;
//			}
//        }
//        if (!duplicate)
//		{
//            nms_centers.push_back(circle_centers[i]);
//            nms_sources.push_back(i < path_a_count ? 1 : 2);
//        }
//    }
//    circle_centers = nms_centers;
//
//    // �����������ĵ���ͼ����ɫΪ A����ɫΪ B��
//    if (1)
//	{
//        cv::Mat color_result;
//        if (img_in.channels() == 1) cv::cvtColor(img_in, color_result, cv::COLOR_GRAY2BGR);
//        else color_result = img_in.clone();
//        for (size_t i = 0; i < circle_centers.size(); ++i)
//		{
//            const cv::Scalar color = nms_sources[i] == 1 ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
//            cv::circle(color_result, circle_centers[i], 1, color, -1);
//            cv::drawMarker(color_result, circle_centers[i], color, cv::MARKER_CROSS, 4, 1);
//        }
//        cv::imwrite("0centers_result.jpg", color_result, { cv::IMWRITE_JPEG_QUALITY, 90 });
//    }
//
//    is_initialized = !circle_centers.empty();
//    filterOutlies_Debscan(circle_centers, results, 
//		                  AppConfig::Instance().limits.debscan_filter_dist_max,
//						  config.debscan_min_pts);
//
//    if (results.size() < 4) 
//	{
//        std::vector<cv::Point2f> results_2;
//        filterOutliers(circle_centers, results, 2.0);
//        filterOutliers(results, results_2, 1.5);
//        results.clear();
//        filterOutliers(results_2, results, 1.0);
//    }
//    LB_COUT << "circle_centers:              " << circle_centers.size() << std::endl;
//    LB_COUT << "results:                     " << results.size() << std::endl;
//	for (size_t i = 0; i < results.size(); ++i)
//	{
//		LB_COUT << results[i].x << "," << results[i].y << "," << 0.0f << std::endl;
//	}
//
//    return is_initialized;
//}



/**************************************************************************************
*��  �ܣ�����������ȡ
*��  ����
*       img                         I         ����ĸ߷ֱ���ͼ��
*       approx_pos                  I         ����ȡ�õ������ؼ���������
*����ֵ�������ؾ��޺����������
*��  ע���Ҷ����ķ���TODO: ��Բ��ϡ����׾ط�
**************************************************************************************/
cv::Point2f MarkPointDetector::RefineSubpixel(const cv::Mat &img,
	                                          cv::Point2f   approx_pos)
{
	// 1. ���þ��޲���
	const int radius = 5;        // ���޴��ڰ뾶�����ڴ�СΪ (2*radius + 1)
	const int x0 = cvRound(approx_pos.x);
	const int y0 = cvRound(approx_pos.y);

	// 2. ȷ���������ڱ߽磬��ֹԽ��
	int left   = x0 - radius > 0 ? x0 - radius : 0;
	int top    = y0 - radius > 0 ? y0 - radius : 0;
	int right  = x0 + radius < img.cols - 1 ? x0 + radius : img.cols - 1;
	int bottom = y0 + radius < img.rows - 1 ? y0 + radius : img.rows - 1;

	double sum_gray = 0;         // �Ҷ�ֵ�ܺ�
	double sum_x = 0;            // x�����Ȩ�ܺ�
	double sum_y = 0;            // y�����Ȩ�ܺ�

	// 3. �������ƣ����㴰���ڵĻҶ���ֵ
	// ��ҵ�����£���ǵ�ͨ���ȱ�����������ֻȡ�Ҷ�ֵ����һ����ֵ������
	// ����򵥲��ù̶���ֵ������Ӧ���㴰������С/ƽ���Ҷ�
	uchar threshold = 80;        // ����ֵ������ʵ�ʷ����������

	// 4. �������ڼ�������
	for (int i = top; i <= bottom; ++i)
	{
		// ʹ��ָ��������ط���
		const uchar* ptr = img.ptr<uchar>(i);
		for (int j = left; j <= right; ++j)
		{
			uchar gray = ptr[j];
			if (gray > threshold)
			{
				// ��ȥ��ֵ������ʹ��ƽ����Ȩ�����Խ�һ����߿���������������
				double weight = (double)gray;
				sum_gray += weight;
				sum_x += weight * j;
				sum_y += weight * i;
			}
		}
	}

	// 5. ��������������λ��
	if (sum_gray > 0)
	{
		return cv::Point2f((float)(sum_x / sum_gray), (float)(sum_y / sum_gray));
	}
	else
	{
		// ���������û���κε������ֵ������ԭλ��
		return approx_pos;
	}
}

/**************************************************************************************
*��  �ܣ��Ҷ����Բ�ֵ
*��  ����
*       img                         I         ����ĸ߷ֱ���ͼ��
*       x                           I         x����
*       y                           I         y����
*����ֵ���Ҷ�ֵ
*��  ע��
**************************************************************************************/
float MarkPointDetector::GetSubpixelGray(const cv::Mat& img, float x, float y)
{
	int x1 = floor(x);
	int y1 = floor(y);
	int x2 = x1 + 1;
	int y2 = y1 + 1;

	if (x1 < 0 || x2 >= img.cols || y1 < 0 || y2 >= img.rows) 
		return 0;

	float dx = x - x1;
	float dy = y - y1;

	float val = (1 - dx) * (1 - dy) * img.at<uchar>(y1, x1) +
		        dx * (1 - dy) * img.at<uchar>(y1, x2) +
		        (1 - dx) * dy * img.at<uchar>(y2, x1) +
		        dx * dy * img.at<uchar>(y2, x2);
	return val;
}

// ��Ե�����߲�ֵ+��Բ��ϣ����ʺϽ��������ı�ǵ㣬TODO����֤����
//cv::Point2f MarkPointDetector::RefineCenter(const cv::Mat& img, cv::Point2f approx_pos, float radius,
//  		                                    cv::Mat K, cv::Mat distCoeffs)
//{
//	// 1. �߽籣����ȷ���ֲ� ROI ��Խ��
//	int roi_size = cvRound(radius * 2.0f + 10.0f); // �ʵ��ؿ�����������������Ե
//	int x0 = cvRound(approx_pos.x);
//	int y0 = cvRound(approx_pos.y);
//
//	int left = std::max(0, x0 - roi_size);
//	int top = std::max(0, y0 - roi_size);
//	int right = std::min(img.cols - 1, x0 + roi_size);
//	int bottom = std::min(img.rows - 1, y0 + roi_size);
//
//	if (right - left < 10 || bottom - top < 10)
//	{
//		return approx_pos; // ROI ̫С�����شֶ�λ
//	}
//
//	// 2. ��ȡ�߷ֱ��ʾֲ�ͼ��
//	cv::Rect roi_rect(left, top, right - left, bottom - top);
//	cv::Mat roi_img = img(roi_rect);
//
//	// 3. �ֲ�����Ӧ��ֵ�ָ��Ե
//	cv::Mat bin_roi;
//	double max_val = 0, min_val = 0;
//	cv::minMaxLoc(roi_img, &min_val, &max_val);
//
//	// ����Աȶ�̫�ͣ�˵��������Ч�����ǵ�
//	if (max_val - min_val < 30.0)
//	{
//		return approx_pos;
//	}
//
//	// ����������������Ӧ�ָ���ֵ��ȡ������С�Ҷȵ��м�ƫ�ϣ��ܿ����ɹ��Σ�
//	double thresh = min_val + (max_val - min_val) * 0.45;
//	cv::threshold(roi_img, bin_roi, thresh, 255, cv::THRESH_BINARY);
//
//	// 4. ��ȡ�߾�������
//	std::vector<std::vector<cv::Point>> contours;
//	cv::findContours(bin_roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
//
//	if (contours.empty())
//	{
//		return approx_pos;
//	}
//
//	// Ѱ���� ROI �������������
//	cv::Point2f roi_center_pt(roi_img.cols / 2.0f, roi_img.rows / 2.0f);
//	int target_contour_idx = -1;
//	double min_dist_to_center = DBL_MAX;
//
//	for (size_t i = 0; i < contours.size(); ++i)
//	{
//		if (contours[i].size() < 6) continue; // ����̫���޷������Բ
//
//		cv::Moments mu = cv::moments(contours[i]);
//		if (mu.m00 < 1.0) continue;
//		cv::Point2f c(mu.m10 / mu.m00, mu.m01 / mu.m00);
//		double d = cv::norm(c - roi_center_pt);
//		if (d < min_dist_to_center)
//		{
//			min_dist_to_center = d;
//			target_contour_idx = i;
//		}
//	}
//
//	if (target_contour_idx == -1)
//	{
//		return approx_pos;
//	}
//
//	const auto& raw_contour = contours[target_contour_idx];
//
//	// 5. ���ģ��ؾ������һ�׵������������߲�ֵ����ȡ�������ر�Ե�㡱
//	// �ò�����ȫ�˷��˽�����ɢ������ı�Եģ������
//	std::vector<cv::Point2f> subpixel_edge_pnts;
//	subpixel_edge_pnts.reserve(raw_contour.size());
//
//	// ��������������
//	cv::Moments target_mu = cv::moments(raw_contour);
//	cv::Point2f coarse_roi_center(target_mu.m10 / target_mu.m00, target_mu.m01 / target_mu.m00);
//
//	for (const auto& p : raw_contour)
//	{
//		// �����Բ��ָ���Ե��ľ���������
//		float dx = p.x - coarse_roi_center.x;
//		float dy = p.y - coarse_roi_center.y;
//		float len = std::sqrt(dx * dx + dy * dy);
//		if (len < 1e-5f) continue;
//
//		float ux = dx / len;
//		float uy = dy / len;
//
//		// �ھ������ϣ�ǰ�������2���㣨��5���� [-2, -1, 0, 1, 2] ���ؾ��룩����һ��΢������
//		float g_prev2 = GetSubpixelGray(roi_img, p.x - 2.0f * ux, p.y - 2.0f * uy);
//		float g_prev1 = GetSubpixelGray(roi_img, p.x - 1.0f * ux, p.y - 1.0f * uy);
//		float g_curr = GetSubpixelGray(roi_img, p.x, p.y);
//		float g_next1 = GetSubpixelGray(roi_img, p.x + 1.0f * ux, p.y + 1.0f * uy);
//		float g_next2 = GetSubpixelGray(roi_img, p.x + 2.0f * ux, p.y + 2.0f * uy);
//
//		// �����ݶ�ֵ
//		float grad_prev = std::abs(g_curr - g_prev2);
//		float grad_curr = std::abs(g_next1 - g_prev1);
//		float grad_next = std::abs(g_next2 - g_curr);
//
//		// ���������������ֵѰ���ݶȼ�ֵ�㣨��������������������Ե��
//		float delta = 0.0f;
//		float denom = 2.0f * (grad_prev - 2.0f * grad_curr + grad_next);
//		if (std::abs(denom) > 1e-4f)
//		{
//			delta = (grad_prev - grad_next) / denom;
//		}
//
//		// ����Խ��ƫ���ֹ��������
//		if (std::abs(delta) < 1.0f)
//		{
//			subpixel_edge_pnts.push_back(cv::Point2f(p.x + delta * ux, p.y + delta * uy));
//		}
//		else
//		{
//			subpixel_edge_pnts.push_back(cv::Point2f(p.x, p.y));
//		}
//	}
//
//	if (subpixel_edge_pnts.size() < 6)
//	{
//		return approx_pos;
//	}
//
//	// 6. ��ϸ߾�����Բ�������������
//	cv::RotatedRect fitted_ellipse = cv::fitEllipse(subpixel_edge_pnts);
//
//	// 7. ת���ظ߷ֱ�����ͼ����ϵ
//	cv::Point2f refined_center_in_img(fitted_ellipse.center.x + left, fitted_ellipse.center.y + top);
//
//	return refined_center_in_img;
//}

// �÷�����Խ����⣬�����ع������Ƿ����ǵ㵼�����Ķ�λ��׼ȷ�����⣬�Ľ���������������
// �ռ䣨xy����˹��Ȩ + �ֲ���̬��Χ��ֵ(�Ҷ�ֵ)�ĵ���ʽ�Ҷ����ķ�
cv::Point2f MarkPointDetector::RefineCenter(const cv::Mat& img, cv::Point2f approx_pos, float radius,
		                                    cv::Mat K, cv::Mat distCoeffs)
{
	cv::Point2f center = approx_pos;
	const int max_iters = 10; // ��������

	// 1. ����΢��ԣ�������ڣ���������Ӧ��ֵ
	// ����뾶��Ϊ 1.5 * radius��ȷ�����ȶ��ɼ��������ı����Ҷ�
	int roi_r = cvRound(radius * 1.5f);
	int x_min = std::max(0, cvRound(approx_pos.x - roi_r));
	int x_max = std::min(img.cols - 1, cvRound(approx_pos.x + roi_r));
	int y_min = std::max(0, cvRound(approx_pos.y - roi_r));
	int y_max = std::min(img.rows - 1, cvRound(approx_pos.y + roi_r));

	float max_val = 0.0f;
	float min_val = 255.0f;
	for (int y = y_min; y <= y_max; ++y)
	{
		for (int x = x_min; x <= x_max; ++x)
		{
			float val = img.at<uchar>(y, x);
			if (val > max_val) 
				max_val = val;
			if (val < min_val) 
				min_val = val;
		}
	}

	// ��̬��ֵ���ų�������������Ч��ߣ�ȡ 35% �Ķ�̬��Χ��
	float thresh = min_val + 0.35f * (max_val - min_val);

	// ����Աȶ�̫�ͣ�˵�������쳣�����س�ʼֵ
	if (max_val - min_val < 15.0f)
	{
		return approx_pos;
	}

	// 2. ������������
	// ��˹�ռ䴰�ڵı�׼�� sigma ��Ϊ radius / 1.5����֤��Եƽ��˥��
	double sigma = radius / 1.5;
	double double_sigma_sq = 2.0 * sigma * sigma;

	for (int iter = 0; iter < max_iters; ++iter)
	{
		double sum_wx = 0.0;
		double sum_wy = 0.0;
		double sum_w = 0.0;

		// �����뾶����Ϊ 1.5 * radius
		int search_r = cvRound(radius * 1.5f);
		int cur_x_min = std::max(0, cvRound(center.x - search_r));
		int cur_x_max = std::min(img.cols - 1, cvRound(center.x + search_r));
		int cur_y_min = std::max(0, cvRound(center.y - search_r));
		int cur_y_max = std::min(img.rows - 1, cvRound(center.y + search_r));

		for (int y = cur_y_min; y <= cur_y_max; ++y)
		{
			for (int x = cur_x_min; x <= cur_x_max; ++x)
			{
				double dx = x - center.x;
				double dy = y - center.y;
				double dist_sq = dx * dx + dy * dy;

				if (dist_sq <= (double)(search_r * search_r))
				{
					double gray = img.at<uchar>(y, x);
					if (gray > thresh)
					{
						// �Ҷ�ǿ��Ȩ��
						double w_int = gray - thresh;
						// �ռ��˹Ȩ�أ�Խ������ǰ����Ȩ��Խ������ƽ��˥��
						double w_spa = exp(-dist_sq / double_sigma_sq);

						// ����Ȩ��
						double w = w_int * w_spa;

						sum_wx += x * w;
						sum_wy += y * w;
						sum_w += w;
					}
				}
			}
		}

		if (sum_w <= 1e-5) break;

		cv::Point2f new_center((float)(sum_wx / sum_w), (float)(sum_wy / sum_w));

		// ���������ж�����ֵС�� 0.005 ����ʱ��Ϊ������
		if (cv::norm(new_center - center) < 0.005f)
		{
			center = new_center;
			break;
		}
		center = new_center;
	}

	return center;
}


//// ��ͨ�����Ҷ����ķ�,������������ά�ؽ�����
//cv::Point2f MarkPointDetector::RefineCenter(const cv::Mat& img, cv::Point2f approx_pos, float radius,
//	                                        cv::Mat K, cv::Mat distCoeffs)
//{
//	cv::Point2f center = approx_pos;
//	const int max_iters = 8;
//	const float r2 = radius * radius;
//
//	for (int iter = 0; iter < max_iters; ++iter)
//	{
//		double sum_wx = 0.0;
//		double sum_wy = 0.0;
//		double sum_w = 0.0;
//
//		int x_min = cvCeil(center.x - radius);
//		int x_max = cvFloor(center.x + radius);
//		int y_min = cvCeil(center.y - radius);
//		int y_max = cvFloor(center.y + radius);
//
//		// �߽籣��
//		x_min = std::max(0, x_min);
//		x_max = std::min(img.cols - 1, x_max);
//		y_min = std::max(0, y_min);
//		y_max = std::min(img.rows - 1, y_max);
//
//		// 1. �ֲ�������Ѱ�ұ�����׼��ȡ��Ե���ֵĻҶ���λ�����ֵ��Ϊ������
//		// �����Ϊ���������ڵ���ͻҶ�ֵ��Ϊ������ֵ����������������Ӱ��
//		float min_val = 255.0f;
//		for (int y = y_min; y <= y_max; ++y)
//		{
//			for (int x = x_min; x <= x_max; ++x)
//			{
//				float dx = x - center.x;
//				float dy = y - center.y;
//				if (dx*dx + dy*dy <= r2)
//				{
//					min_val = std::min(min_val, (float)img.at<uchar>(y, x));
//				}
//			}
//		}
//
//		float bg_threshold = min_val + 5.0f; // �Ը��ڱ�������
//
//		// 2. �����Ȩ����
//		for (int y = y_min; y <= y_max; ++y)
//		{
//			for (int x = x_min; x <= x_max; ++x)
//			{
//				float dx = x - center.x;
//				float dy = y - center.y;
//				if (dx*dx + dy*dy <= r2)
//				{
//					float gray = img.at<uchar>(y, x);
//					if (gray > bg_threshold)
//					{
//						// Ȩ�ؿ���ʹ�� (gray - bg_threshold)
//						double w = gray - bg_threshold;
//						sum_wx += x * w;
//						sum_wy += y * w;
//						sum_w += w;
//					}
//				}
//			}
//		}
//
//		if (sum_w <= 0.0) break;
//
//		cv::Point2f new_center((float)(sum_wx / sum_w), (float)(sum_wy / sum_w));
//
//		// ������ļ������ٷ����ƶ�������Ϊ����
//		if (cv::norm(new_center - center) < 0.01f)
//		{
//			center = new_center;
//			break;
//		}
//		center = new_center;
//	}
//
//	return center;
//}


///**************************************************************************************
//*��  �ܣ������ز�ֵ
//*��  ����
//*       img                         I         ����ĸ߷ֱ���ͼ��
//*       x                           I         x����
//*       y                           I         y����
//*����ֵ���Ҷ�ֵ
//*��  ע����ҵ�����ޣ��ݶ����ķ� + ³���Թ��ˣ��Կɼ����Լ����ع����Ľ�������Ч
//**************************************************************************************/
//cv::Point2f MarkPointDetector::RefineCenter(const cv::Mat& img, cv::Point2f approx_pos, float radius,
//	                                        cv::Mat K, cv::Mat distCoeffs)
//{
//	const int num_rays = 72;           // ���Ӳ����ܶ� (ÿ5��һ��)
//	const float search_range = 6.0f;   // �Դ��������Χȷ�����Ǳ�Ե
//	const float step = 0.5f;
//
//	std::vector<cv::Point2f> edge_points;
//	std::vector<float> edge_weights;   // �洢�ݶ�ǿ����Ϊ���Ȩ��
//
//	for (int i = 0; i < num_rays; ++i)
//	{
//		float angle = i * (2.0f * (float)CV_PI / num_rays);
//		float dx = cos(angle);
//		float dy = sin(angle);
//
//		std::vector<float> profile;
//		// 1. ��������
//		for (float r = radius - search_range; r <= radius + search_range; r += step)
//		{
//			profile.push_back(GetSubpixelGray(img, approx_pos.x + r * dx, approx_pos.y + r * dy));
//		}
//
//		// 2. ����һ�ײ���ݶ� (ʹ���Դ�����ӿ���)
//		std::vector<float> grads;
//		float max_g = -1;
//		int max_idx = -1;
//		for (int j = 2; j < (int)profile.size() - 2; ++j)
//		{
//			// ʹ�� [j-2, j-1, j+1, j+2] �����ݶȣ��ȼ򵥲�ָ���
//			float g = abs(profile[j - 2] + profile[j - 1] - profile[j + 1] - profile[j + 2]);
//			grads.push_back(g);
//			if (g > max_g) {
//				max_g = g;
//				max_idx = j - 2; // ��Ӧgrads������
//			}
//		}
//
//		// 3. ���ĸĽ����ݶ����ķ���λ (�������߲�ֵ��׼)
//		// ȡ�ݶȷ�ֵ�������Ҹ�2���㣬��������
//		if (max_idx >= 2 && max_idx < (int)grads.size() - 2 && max_g > 10.0f)
//		{
//			double sum_gr = 0, sum_g = 0;
//			for (int k = max_idx - 2; k <= max_idx + 2; ++k)
//			{
//				float weight = grads[k];
//				float r_val = (radius - search_range) + (k + 2) * step; // ���㵱ǰ�İ뾶����
//				sum_gr += (double)weight * r_val;
//				sum_g += (double)weight;
//			}
//
//			if (sum_g > 0)
//			{
//				float best_r = (float)(sum_gr / sum_g);
//				edge_points.push_back(cv::Point2f(approx_pos.x + best_r * dx,
//					approx_pos.y + best_r * dy));
//				edge_weights.push_back(max_g);
//			}
//		}
//	}
//
//	if (edge_points.size() < 10)
//	{
//		return approx_pos;
//	}
//
//	if (0)
//	{
//		cv::Mat debugImg;
//		if (img.channels() == 1)
//		{
//			cv::cvtColor(img, debugImg, cv::COLOR_GRAY2BGR);
//		}
//		else
//		{
//			debugImg = img.clone();
//		}
//
//		const float alpha = 1.0f; // ���ò�͸����ɫ��ȷ�ϵ�ȷʵ������
//
//		for (const auto& p : edge_points)
//		{
//			cv::circle(debugImg, p, 1, cv::Scalar(0, 0, 255), -1); // ����ɫʵ��ԭ��
//		}
//
//		bool ok = cv::imwrite("0lineEllipse.png", debugImg);
//	}
//
//	// 4. �쳣ֵ���ˣ�������Ϻ��޳�������ĵ� (��ֹ���۵����)
//	std::vector<cv::Point2f> undistorted_points;
//	if (!K.empty() && !distCoeffs.empty())
//	{
//		cv::undistortPoints(edge_points, undistorted_points, K, distCoeffs, cv::noArray(), K);
//	}
//	else
//	{
//		undistorted_points = edge_points;
//	}
//
//	// ��һ�����
//	cv::RotatedRect ell = cv::fitEllipseDirect(undistorted_points);
//
//	// ³�����޳�������ÿ���㵽��Բ��Ե�ľ��룬�޳�ƫ�����ĵ�
//	std::vector<cv::Point2f> final_points;
//	for (const auto& p : undistorted_points)
//	{
//		// �򻯵ľ����飺�����ĵľ�����ƽ���뾶�Ա�
//		float dist = cv::norm(p - ell.center);
//		float expected_r = (ell.size.width + ell.size.height) / 4.0f;
//		if (abs(dist - expected_r) < 2.0f)       // ֻ��������ƫ��С��2���صĵ�
//		{
//			final_points.push_back(p);
//		}
//	}
//
//	if (final_points.size() < 10)
//	{
//		return ell.center;
//	}
//
//	// 5. �������
//	cv::RotatedRect final_ell = cv::fitEllipseDirect(final_points);
//	return final_ell.center;
//}


// ����������ά��֮���ŷ�Ͼ���
double MarkPointDetector::calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2)
{
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	return std::sqrt(dx * dx + dy * dy);
}

// ����㼯�ļ������ĵ�
cv::Point2f MarkPointDetector::calculateCentroid(const std::vector<cv::Point2f>& points)
{
	if (points.empty()) 
	{
		return cv::Point2f(0, 0);
	}

	double sum_x = 0.0, sum_y = 0.0;
	for (const auto& p : points) 
	{
		sum_x += p.x;
		sum_y += p.y;
	}

	return cv::Point2f(sum_x / points.size(), sum_y / points.size());
}

// ������Ⱥ��
// ����˵����
//   points: �����ԭʼ�㼯
//   std_factor: ��׼��ϵ��������2��3��ֵԽ�����Խ���ɣ�
// ����ֵ�����˺�ĵ㼯
// �Ż���
// 1)����ʽ���ȹ��˵����Ե���Ⱥ�㣬�����¼������ĺ���ֵ���ظ� 2-3 �Σ������ʼ��Ⱥ��Ӱ�����ļ���
// 2)ԭʼ�㷨����㼯Χ�Ƽ���������̬�ֲ�����ʵ�ʳ����п��ܲ���������������Ż������÷�λ�������ķ�λ�� IQR��
// ���ó������㼯�ֲ�����̬��������ȷֲ���ƫ̬�ֲ���
// �����߼���
// �������о�����ķ�λ�� Q1��25%����Q3��75%��
// �����ķ�λ�� IQR = Q3 - Q1
// ��ֵ = Q3 + 1.5 * IQR������ IQR ��Ⱥ���ж�����
int MarkPointDetector::filterOutliers(const std::vector<cv::Point2f> &points,
                                std::vector<cv::Point2f>       &filtered_points,
	                            double                          std_factor)
{
	if (points.size() <= 1) 
	{
		filtered_points.clear();
		for (size_t i = 0; i < points.size(); ++i)
		{
			filtered_points.push_back(points[i]);
		}
		return 0; // ��̫���������
	}

	// 1. �������ĵ�
	 cv::Point2f centroid = calculateCentroid(points);

	// 2. ����ÿ���㵽���ĵ�ľ���
	std::vector<double> distances;
	distances.reserve(points.size());
	for (const auto& p : points)
	{
		distances.push_back(calculateDistance(p, centroid));
	}

	// 3. �������ľ�ֵ
	double mean_distance = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();

	// 4. �������ı�׼��
	double sum_squared_diff = 0.0;
	for (double d : distances) 
	{
		sum_squared_diff += std::pow(d - mean_distance, 2);
	}
	double std_distance = std::sqrt(sum_squared_diff / distances.size());

	// 5. ������ֵ����ֵ + std_factor * ��׼�
	double threshold = mean_distance + std_factor * std_distance;

	// 6. ������Ⱥ��
	for (size_t i = 0; i < points.size(); ++i) 
	{
		if (distances[i] <= threshold)
		{
			filtered_points.push_back(points[i]);
		}
	}

	//// ���������Ϣ����ѡ��
	//LB_COUT << "=== ��Ⱥ�������Ϣ ===" << std::endl;
	//LB_COUT << "ԭʼ����: " << points.size() << std::endl;
	//LB_COUT << "�Ƴ�����Ⱥ����: " << points.size() - filtered_points.size() << std::endl;
	//LB_COUT << "���˺����: " << filtered_points.size() << std::endl;
	//LB_COUT << "������ֵ: " << threshold << std::endl;
	return 0;
}



// ���ǵ���ǵ�ľۼ��Ժ;����ԣ�ʹ��descan�����˲�
// points             ����ĵ㼯��cv::Point2f ��ʽ��
// filtered_points    �˲���ĵ㼯��cv::Point2f ��ʽ��
// eps                ����뾶�������㱻��Ϊ�ھӵ�������ؾ��룩
// minPts             ���ٵ��������ڴ������ĵ㽫����Ϊ������
bool MarkPointDetector::filterOutlies_Debscan(const std::vector<cv::Point2f> &points,
	                                     std::vector<cv::Point2f>       &filtered_points,
						                 float eps,
						                 int minPts)
{
	if (points.empty())
	{
		return false;
	}

	float eps2 = eps * eps;
	int n = points.size();
	std::vector<int> labels(n, -1); // -1: δ����, 0: ����, >0: ��ID
	int clusterId = 0;

	// 1. ִ�� DBSCAN ����
	for (int i = 0; i < n; i++) 
	{
		if (labels[i] != -1)
		{
			continue;
		}

		// Ѱ���ھ�
		std::vector<int> neighbors;
		for (int j = 0; j < n; j++) 
		{
			if ((powf((points[i].x - points[j].x), 2) + powf((points[i].y - points[j].y), 2)) <= eps2)
			{
				neighbors.push_back(j);
			}
		}

		if (neighbors.size() < (size_t)minPts) 
		{
			labels[i] = 0; // ���Ϊ����
		}
		else 
		{
			clusterId++;
			labels[i] = clusterId;

			// ��չ�� (ʹ�ö���ģ��ݹ�)
			std::vector<int> seeds = neighbors;
			for (size_t k = 0; k < seeds.size(); k++) 
			{
				int currIdx = seeds[k];
				if (labels[currIdx] == 0)
				{
					labels[currIdx] = clusterId; // �������߽��
				}
				if (labels[currIdx] != -1)
				{
					continue;
				}

				labels[currIdx] = clusterId;
				std::vector<int> currNeighbors;
				for (int j = 0; j < n; j++) 
				{
					
					if ((powf((points[currIdx].x - points[j].x), 2) + powf((points[currIdx].y - points[j].y), 2)) <= eps2)
					{
						currNeighbors.push_back(j);
					}
				}
				if (currNeighbors.size() >= (size_t)minPts)
				{
					seeds.insert(seeds.end(), currNeighbors.begin(), currNeighbors.end());
				}
			}
		}
	}

	// 2. ͳ���ĸ��صĵ�����࣬�ۼ���
	if (clusterId == 0)
	{
		return false; // ȫ������
	}

	std::vector<int> counts(clusterId + 1, 0);
	for (int k : labels) 
	{
		if (k > 0)
		{
			counts[k]++;
		}
	}

	auto maxIt = std::max_element(counts.begin() + 1, counts.end());
	int targetId = std::distance(counts.begin(), maxIt);
	int maxPointsCount = *maxIt;

	// 3. ����Ŀ��ص�����
	cv::Point2f sum(0, 0);
	filtered_points.clear();
	for (int i = 0; i < n; i++)
	{
		if (labels[i] == targetId) 
		{
			filtered_points.push_back(points[i]);
		}
	}

	return true;
}


// ����������������ӳ�䵽����
inline int FastGeoHash::getIdx(float len) const
{
	if (step < 1e-8f)
	{ 
		return -1;
	}

	int idx = (int)(len / step);    // step:ÿ��Ͱ�ĳ���
	if (idx < 0)
	{
		return 0;
	}
	if (idx >= L_BINS)
	{
		return L_BINS - 1;
	}
	return idx;
}

// ��������������3D�����������Ⱥ�����ֵ��
// ע�⣺����ֱ�ӷ��س��ȣ���Ϊդ���ǻ��ڳ���mm���ֵ�
inline bool FastGeoHash::calcFeature(const cv::Point3f &A,
		                             const cv::Point3f &B,
							         const cv::Point3f &C,
		                             float& lAB, 
							         float& lAC, 
							         float& cosA) const
{
	float abx = B.x - A.x, aby = B.y - A.y, abz = B.z - A.z;
	float acx = C.x - A.x, acy = C.y - A.y, acz = C.z - A.z;

	float d2AB = abx * abx + aby * aby + abz * abz;
	// ��ֵ��飺��� AB ̫����ֱ������
	if (d2AB < minDistanceSq)
	{
		return false;
	}
	float d2AC = acx * acx + acy * acy + acz * acz;
	// ��ֵ��飺��� AB ̫����ֱ������
	if (d2AC < minDistanceSq)
	{
		return false;
	}
	lAB = std::sqrt(d2AB);
	lAC = std::sqrt(d2AC);

	float dot = abx * acx + aby * acy + abz * acz;
	cosA = dot / (lAB * lAC + 1e-8f);

	return true;
} 

// --- ��һ���֣����߽��� ---
// ����130��ģ�͵㣬������ϣ��
int FastGeoHash::build()
{
	int N = template_pnts.size();
	if (N < 3)
	{
		return 401;
	}

	// 1. ��һ�ֱ�����ͳ��ÿ��Ͱ�Ĵ�С
	std::memset(counts, 0, sizeof(int) * L_BINS * L_BINS);
	for (int i = 0; i < N; ++i)              // ��A
	{
		for (int j = 0; j < N; ++j)          // ��B
		{ 
			if (i == j)
			{
				continue;
			}

			for (int k = j + 1; k < N; ++k)  // ��C (j+1 ��֤BC��ϲ��ظ�)
			{ 
				if (i == k)
				{
					continue;
				}


				float l1, l2, c;
				if (!calcFeature(template_pnts[i], template_pnts[j], template_pnts[k], l1, l2, c))
				{
					continue;              // ����̫�������Ը�����
				}
				if (l1 > l2)
				{
					std::swap(l1, l2);
				}
				int key = getIdx(l1) * L_BINS + getIdx(l2);            // ��l1��l2���ɵĶ�άդ��Ͱ
				counts[key]++;
			}
		}
	}

	// 2. ����ƫ���� (ǰ׺��)
	offsets[0]       = 0;
	int totalEntries = counts[0];
	int total_c      = L_BINS * L_BINS;
	for (int i = 1; i < total_c; ++i)
	{
		offsets[i]    = offsets[i - 1] + counts[i - 1];
		totalEntries += counts[i];
	}

	// 3. �ڶ��ֱ�����������ʵ����
	entries.resize(totalEntries);
	int* currentPos = new int[L_BINS * L_BINS];
	std::memcpy(currentPos, offsets, sizeof(int) * L_BINS * L_BINS);
	for (int i = 0; i < N; ++i)
	{
		for (int j = 0; j < N; ++j)
		{
			if (i == j)
			{
				continue;
			}
			for (int k = j + 1; k < N; ++k)
			{
				if (i == k)
				{
					continue;
				}
				float l1, l2, c;
				if (!calcFeature(template_pnts[i], template_pnts[j], template_pnts[k], l1, l2, c))
				{
					continue;
				}
				if (l1 > l2)
				{
					std::swap(l1, l2);
				}
				int key = getIdx(l1) * L_BINS + getIdx(l2);
				entries[currentPos[key]++] = { l1, l2, std::acos(ClampCos(c)), (uint8_t)i };
			}
		}
	}
	delete[] currentPos;
	LB_COUT << "Build finished. Total combinations: " << totalEntries << std::endl;
	return 0;
}

// --- �ڶ����֣����߲�ѯ ---
// ���볡���е������� A, B, C�����������ν���ͶƱ
int FastGeoHash::addVote(const cv::Point3f &sA,
		                 const cv::Point3f &sB,
			             const cv::Point3f &sC,
			             float             angleTolerance,
				         float             lengthTolerance,
				         int               *valid_count)
{
	if (step < 1e-8f || angleTolerance <= 0.0f || lengthTolerance <= 0.0f)
	{
		return -1;
	}

	int count_t   = (*valid_count);
	bool is_valid = false;
	float l1, l2, targetCosA;
	if (!calcFeature(sA, sB, sC, l1, l2, targetCosA))
	{
		return -1;                      // ������̫�����޷������Ƚ�����
	}
	if (l1 > l2)
	{
		std::swap(l1, l2);
	}
	float targetAngleA = std::acos(ClampCos(targetCosA));
	int i1 = getIdx(l1);
	int i2 = getIdx(l2);
	int binRadius = std::max(1, (int)std::ceil(lengthTolerance / step));

	// �ھ������������뾶�ɳ������ƫ������������ݲ����1��Ͱʱ©ƥ��
	for (int di = -binRadius; di <= binRadius; ++di)
	{
		for (int dj = -binRadius; dj <= binRadius; ++dj)
		{
			int ni = i1 + di;
			int nj = i2 + dj;
			if (ni < 0 || ni >= L_BINS || nj < 0 || nj >= L_BINS)
			{
				continue;
			}
			int key   = ni * L_BINS + nj;
			int start = offsets[key];
			int end   = start + counts[key];

			// �����ڴ�ɨ�裺Ϊ������targetCosA�ҵ���Ӧ��
			for (int k = start; k < end; ++k)
			{
				if (std::abs(entries[k].l1 - l1) > lengthTolerance ||
					std::abs(entries[k].l2 - l2) > lengthTolerance)
				{
					continue;
				}
				if (std::abs(entries[k].angleA - targetAngleA) < angleTolerance)
				{
					int id_t = entries[k].pointIdA;
					if (id_t < 0 || id_t >= (int)votes.size())
					{
						continue;
					}
					votes[id_t]++;
					// ����ͶƱ��Ч����������Ͷ���˼���Ʊ
					// ���ܴ��ں�ѡ���һ�ײ�����L1,L2,cosA����ģ�����ܶ�Ӧ��������Σ�����ÿ��count_tֻ��һ
					is_valid = true;
				}
			}
		}
	}
	if (is_valid)
	{
		count_t++;
	}

	*valid_count = count_t;

	return 0;
}

//// count - ��Ʊ��
//// minPercent - ѡ��id����Сռ��
//int FastGeoHash::getResult(int count, float minPercent)
//{
//	// �������Ʊ
//	int bestId = -1;
//	int maxV   = 0;
//	int id     = -1;
//
//	for (int i = 0; i < (int)votes.size(); ++i)
//	{
//		if (votes[i] > maxV)
//		{
//			maxV = votes[i];
//			bestId = i;
//		}
//	}
//
//	if (maxV > 4 && count > 4)            // ��Ʊ������Ϊ5������ͶƱ������Ϊ5�Σ�����������Ӱ��ϴ�
//	{
//		int count_t = (int)(minPercent * count);
//		if (maxV > count_t)
//		{
//			id = bestId;
//		}
//	}
//	if (id < 0)
//	{
//		LB_COUT << "ͶƱ������" << count << "  ����Ʊ����" << maxV <<" No"<< std::endl;
//	}
//	else
//	{
//		LB_COUT << "ͶƱ������" << count << "  ����Ʊ����" << maxV << " �ҵ��˶�Ӧ��" << std::endl;
//	}
//	return id;
//}

int FastGeoHash::getResult(int count, float minPercent)
{
    int bestId = -1;
    int secondId = -1;
    int maxV = 0;
    int secondV = 0;

    for (int i = 0; i < (int)votes.size(); ++i) 
	{
        if (votes[i] > maxV) 
		{
            secondV = maxV;
            secondId = bestId;
            maxV = votes[i];
            bestId = i;
        } 
		else if (votes[i] > secondV)
		{
            secondV = votes[i];
            secondId = i;
        }
    }

    // 1. ����Ʊ������ (���� 5 Ʊ����)
    // 2. �����ԣ��ȵڶ����߳�һ������ (Ratio Test)
    if (maxV >= 6) 
	{
        if (secondV == 0 || (float)maxV / secondV > 1.5f)
		{
            return bestId;
        }
    }
    return -1;
}

// �Ƚ�ʶ���
// sA                Ŀ���
// otherCandidates  �����е�������ά�ؽ���
// angleToleranceDeg     �Ƕ��ݲ�
// minPercent       ���Ʊ��ռ�ȣ�ռ���в�ѯ�����İٷֱ�
int FastGeoHash::query(const cv::Point3f& sA,
		               const std::vector<cv::Point3f>& otherCandidates,
		               float angleToleranceDeg,
			           float minPercent)
{
	// 1. ��ʼ��
	float angleTolerance = DegreesToRadians(angleToleranceDeg);
	if (votes.size() != template_pnts.size())
	{
		votes.assign(template_pnts.size(), 0);
	}
	clearVotes();
	// Ԥ����ƽ��������ֵ��������ѭ���з������� sqrt
	float maxDistSq = maxDistance * maxDistance;

	// 2. ���˵��� sA ̫Զ����Ч�㣬������Ч���
	// �� 400mm �ĳ����£��� A ���� 400mm �ĵ��޷���ɹ�ϣ���ܼ�����������
	std::vector<cv::Point3f> validNeighbors;
	validNeighbors.reserve(otherCandidates.size());

	for (const auto& p : otherCandidates)
	{
		if (p == sA)
		{
			continue;
		}
		float dx = p.x - sA.x;
		float dy = p.y - sA.y;
		float dz = p.z - sA.z;
		float d2 = dx*dx + dy*dy + dz*dz;

		// ֻ���� [minDist, maxDist] ��Χ�ڵĵ��������
		if (d2 <= maxDistSq && d2 >= minDistanceSq)
		{
			validNeighbors.push_back(p);
		}
	}

	if (validNeighbors.size() < 2)
	{
		return -1;
	}

	// 3. ִ�ж���ͶƱ
	// Ϊ�����ܣ���� validNeighbors ̫�ࣨ���糬�� 6 ����������ֻȡǰ 6 ��
	size_t cut_size = AppConfig::Instance().limits.vote_pnt_size_max;
	int    count    = 0;
	if (cut_size > validNeighbors.size())
	{
		cut_size = validNeighbors.size();
	}
	for (size_t i = 0; i < cut_size; ++i)
	{
		for (size_t j = i + 1; j < cut_size; ++j)
		{
			// ����֮ǰ����ĵ���ͶƱ����
			addVote(sA, validNeighbors[i], validNeighbors[j], angleTolerance, lengthTolerance, &count);
		}
	}

	// 4. ��ȡ�������������Ʊ���
	int id_t = getResult(count, minPercent);

	return id_t;
}

void FastGeoHash::clearVotes()
{
	std::fill(votes.begin(), votes.end(), 0);
}

// ������Ƶı任����Rt
int FastGeoHash::computeRigidTransformSVD(const std::vector<cv::Point3f>& src,
	                                      const std::vector<cv::Point3f>& dst,
	                                      cv::Mat &Rt)
{
	int n = src.size();
	if (n < 3)
	{
		return -1;
	}
	// 1. ��������
	cv::Point3f centerSrc(0, 0, 0), centerDst(0, 0, 0);
	for (int i = 0; i < n; i++)
	{
		centerSrc += src[i];
		centerDst += dst[i];
	}
	centerSrc *= (1.0 / n);
	centerDst *= (1.0 / n);

	// 2. ȥ���Ļ�������Э������� H
	cv::Mat H = cv::Mat::zeros(3, 3, CV_64F);
	for (int i = 0; i < n; i++)
	{
		cv::Mat s = (cv::Mat_<double>(3, 1) << src[i].x - centerSrc.x, src[i].y - centerSrc.y, src[i].z - centerSrc.z);
		cv::Mat d = (cv::Mat_<double>(3, 1) << dst[i].x - centerDst.x, dst[i].y - centerDst.y, dst[i].z - centerDst.z);
		H += d * s.t();
	}

	// 3. SVD �ֽ�
	cv::SVD svd(H);
	cv::Mat R = svd.u * svd.vt;

	// 4. �������ʽ����ֹ���־�����
	if (cv::determinant(R) < 0)
	{
		cv::Mat V = svd.vt.t();
		V.col(2) *= -1;         // ��ת���һ��
		R = V.t() * svd.u.t();  // ���¼��� R
		R = R.t();
	}

	// 5. ����ƽ�� t
	cv::Mat cSrc = (cv::Mat_<double>(3, 1) << centerSrc.x, centerSrc.y, centerSrc.z);
	cv::Mat cDst = (cv::Mat_<double>(3, 1) << centerDst.x, centerDst.y, centerDst.z);
	cv::Mat t = cDst - R * cSrc;

	// 6. ���� 4x4 ����
	Rt = cv::Mat::eye(4, 4, CV_64F);
	R.copyTo(Rt.rowRange(0, 3).colRange(0, 3));
	t.copyTo(Rt.rowRange(0, 3).col(3));

	return 0;
}

	// ���ò���
int FastGeoHash::set_template_config(float   minDistance_t,
		                             float   maxDistance_t)
{
	minDistance = minDistance_t;
	maxDistance = maxDistance_t;
	minDistanceSq = minDistance_t * minDistance_t;
	step = maxDistance_t / L_BINS;
	return 0;
}

// ���ò�ѯ����
int FastGeoHash::set_query_config(float   angleToleranceDeg_t,
	                              float   minPercent_t,
	                              float   lengthTolerance_t)
{
	angleTolerance = DegreesToRadians(angleToleranceDeg_t);
	minPercent   = minPercent_t;
	if (lengthTolerance_t > 0.0f)
	{
		lengthTolerance = lengthTolerance_t;
	}
	return 0;
}

// ����ɨ���ǵ�ɨ��ͷ��ǵ�ı궨���
int FastGeoHash::set_scan_to_marker_RT(cv::Mat &scan_to_marker_RT_t)
{
	// �������Ƿ�Ϊ��
	if (scan_to_marker_RT_t.empty())
	{
		std::cerr << "[����] �궨����������Ϊ�գ�" << std::endl;
		return 400; // ����100��ʾ��������
	}

	// �����ʹ�� cv::Mat::clone() ����������ⲿ�����ͷŵ���Ұָ��
	scan_to_marker_RT = scan_to_marker_RT_t.clone();
	return 0;
}

// �õ�ģ���
int FastGeoHash::read_template_pnts(const char *file_name)
{
	FILE          *infile     = NULL;
	char           buff[2048] = { 0 };
	float          x          = 0.0f;
	float          y          = 0.0f;
	float          z          = 0.0f;

	template_pnts.clear();
	int reserve_count = AppConfig::Instance().limits.mark_point_size_max;
	if (reserve_count < 1)
	{
		reserve_count = 1;
	}
	template_pnts.reserve(reserve_count);
	infile = fopen(file_name, "r");
	if (NULL == infile)
	{
		printf("File not found\n");
		return 1;
	}
	while (!feof(infile))
	{
		if (!fscanf(infile, "%f %f %f\n", &x, &y, &z)) // �����س��Ż��� 
		{
			break;
		}
		template_pnts.push_back(cv::Point3f(x, y, z));
	}
	fclose(infile);
	if (template_pnts.size() > (size_t)AppConfig::Instance().limits.mark_point_size_max)
	{
		std::cerr << "[Config] template point count exceeds mark_point_size_max." << std::endl;
		return 402;
	}
	votes.assign(template_pnts.size(), 0);
	return 0;
}

// ����λ�ø��ٺ����ӿ�
// srcPoints         ������ĵ��ƣ�˫Ŀ���ٵı�ǵ���ƣ�
// ���Ծ���Ƕ�Լ����ɨ�����ϱ�ǵ���Ծ���ͽǶȲ��䣨���Ƽ��ι�ϣ�Ķ�ά������ұ���
int FastGeoHash::Get_Track_Pose(std::vector<cv::Point3f>& frame_3d_points,
	                            float angleToleranceDeg,
                                float minPercent)
{
	filtered_frame_3d_points.clear();
	corres_template_points_ID.clear();

	const AppConfig::Limits& limits = AppConfig::Instance().limits;
	size_t pnt_size = frame_3d_points.size();
	if (pnt_size < 3)
	{
		return 400;
	}

	// 1. �˲����Ȱ���ά�����޳�ԶƮ�����ؽ��㣺maxDistance��Χ���ھӲ�����ɾ����
	std::vector<cv::Point3f> valid_frame_points;
	valid_frame_points.reserve(pnt_size);
	float max_dist_sq = maxDistance * maxDistance;
	for (size_t i = 0; i < pnt_size; ++i)
	{
		int neighbor_count = 0;
		for (size_t j = 0; j < pnt_size; ++j)
		{
			if (i == j)
			{
				continue;
			}
			float dx = frame_3d_points[j].x - frame_3d_points[i].x;
			float dy = frame_3d_points[j].y - frame_3d_points[i].y;
			float dz = frame_3d_points[j].z - frame_3d_points[i].z;
			float d2 = dx * dx + dy * dy + dz * dz;
			if (d2 <= max_dist_sq)
			{
				neighbor_count++;
				if (neighbor_count >= limits.recon_neighbor_count_min)
				{
					break;
				}
			}
		}
		if (neighbor_count >= limits.recon_neighbor_count_min)
		{
			valid_frame_points.push_back(frame_3d_points[i]);
		}
	}

	if (valid_frame_points.size() < 3)
	{
		LB_COUT << "��ά����ɸѡ��������������ܽ���λ��: " << valid_frame_points.size() << std::endl;
		return 500;
	}
	LB_COUT << "��ά����ɸѡ������: " << valid_frame_points.size() << " / " << frame_3d_points.size() << std::endl;

	// 2. ����ģ��㼸�ι�ϣʶ��ģ���Ӧ�㡣
	filtered_frame_3d_points.reserve(valid_frame_points.size());
	corres_template_points_ID.reserve(valid_frame_points.size());
	for (size_t ii = 0; ii < valid_frame_points.size(); ii++)
	{
		std::vector<cv::Point3f> otherCandidates;
		otherCandidates.reserve(valid_frame_points.size() - 1);
		for (size_t jj = 0; jj < valid_frame_points.size(); jj++)
		{
			if (ii == jj)
			{
				continue;
			}
			otherCandidates.push_back(valid_frame_points[jj]);
		}

		int id_fit = query(valid_frame_points[ii],
			               otherCandidates,
			               angleToleranceDeg,
			               minPercent);
		if (id_fit >= 0)
		{
			filtered_frame_3d_points.push_back(valid_frame_points[ii]);
			corres_template_points_ID.push_back(id_fit);
		}
	}

	cv::Mat                  Rt;                      // ģ�嵽˫Ŀ����ϵ�任
	std::vector<cv::Point3f> corres_template_pnts;    // ��֡����ͨ��У��ĵ��Ӧ��ģ���
	corres_template_pnts.reserve(corres_template_points_ID.size());
	for (int ii = 0; ii < corres_template_points_ID.size(); ii++)
	{
		int id_temp = corres_template_points_ID[ii];
		corres_template_pnts.push_back(template_pnts[id_temp]);
	}

	int min_pose_points = AppConfig::Instance().limits.vote_filter_pnt_size_min;
	if (min_pose_points < 3)
	{
		min_pose_points = 3;
	}
	if (filtered_frame_3d_points.size() < (size_t)min_pose_points)
	{
		LB_COUT << "ģ��ƥ����������� ���ܽ���λ��: " << filtered_frame_3d_points.size() << std::endl;
		return 500;
	}

	// 3. RANSAC + SVD��ÿ��ȡ3���Ӧ����Rt����ȫ���Ӧ��ͳ���ڵ㡣
	int res_err = 0;
	int best_inlier_count = -1;
	double best_error_sum = DBL_MAX;
	cv::Mat best_Rt;
	const double ransac_dist_thresh = limits.pose_ransac_inlier_dist;
	std::mt19937 gen(12345);
	std::uniform_int_distribution<int> dist_idx(0, (int)filtered_frame_3d_points.size() - 1);
	for (int iter = 0; iter < limits.pose_ransac_iterations; ++iter)
	{
		int idx0 = dist_idx(gen);
		int idx1 = dist_idx(gen);
		int idx2 = dist_idx(gen);
		if (idx0 == idx1 || idx0 == idx2 || idx1 == idx2)
		{
			--iter;
			continue;
		}

		std::vector<cv::Point3f> sample_template_points;
		std::vector<cv::Point3f> sample_frame_points;
		sample_template_points.reserve(3);
		sample_frame_points.reserve(3);
		sample_template_points.push_back(corres_template_pnts[idx0]);
		sample_template_points.push_back(corres_template_pnts[idx1]);
		sample_template_points.push_back(corres_template_pnts[idx2]);
		sample_frame_points.push_back(filtered_frame_3d_points[idx0]);
		sample_frame_points.push_back(filtered_frame_3d_points[idx1]);
		sample_frame_points.push_back(filtered_frame_3d_points[idx2]);

		cv::Mat candidate_Rt;
		res_err = computeRigidTransformSVD(sample_template_points, sample_frame_points, candidate_Rt);
		if (res_err != 0 || candidate_Rt.empty())
		{
			continue;
		}

		int inlier_count = 0;
		double error_sum = 0.0;
		for (size_t i = 0; i < filtered_frame_3d_points.size(); ++i)
		{
			double err = TransformDistance(candidate_Rt, corres_template_pnts[i], filtered_frame_3d_points[i]);
			if (err < ransac_dist_thresh)
			{
				inlier_count++;
				error_sum += err;
			}
		}

		if (inlier_count > best_inlier_count ||
			(inlier_count == best_inlier_count && error_sum < best_error_sum))
		{
			best_inlier_count = inlier_count;
			best_error_sum = error_sum;
			best_Rt = candidate_Rt.clone();
		}
	}

	if (best_Rt.empty() || best_inlier_count < min_pose_points)
	{
		LB_COUT << "RANSAC�ڵ������������ܽ���λ��: " << best_inlier_count << std::endl;
		return 501;
	}

	Rt = best_Rt.clone();
	LB_COUT << " RANSAC Rt��from Template to Vision�� is: " << std::endl;
	LB_COUT << std::fixed << std::setprecision(8);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			LB_COUT << std::setw(8) << Rt.at<double>(i, j) << " ";
		}
		LB_COUT << std::endl;
	}
	LB_COUT << "RANSAC�ڵ������� " << best_inlier_count << std::endl;

	// 4. ����RANSAC Rt����ɸѡ�ڵ㣬����ȫ���ڵ���һ��SVD�õ�����Rt��
	std::vector<cv::Point3f> new_filtered_frame_3d_points;
	std::vector<cv::Point3f> new_corres_template_points;
	//std::vector<int> new_corres_template_points_ID;
	new_filtered_frame_3d_points.reserve(filtered_frame_3d_points.size());
	new_corres_template_points.reserve(corres_template_pnts.size());
	//new_corres_template_points_ID.reserve(corres_template_points_ID.size());
	for (size_t i = 0; i < filtered_frame_3d_points.size(); ++i)
	{
		double err = TransformDistance(Rt, corres_template_pnts[i], filtered_frame_3d_points[i]);
		if (err < ransac_dist_thresh)
		{
			new_corres_template_points.push_back(corres_template_pnts[i]);
			new_filtered_frame_3d_points.push_back(filtered_frame_3d_points[i]);
			//new_corres_template_points_ID.push_back(corres_template_points_ID[i]);
		}
	}

	if (new_filtered_frame_3d_points.size() < (size_t)min_pose_points)
	{
		LB_COUT << "�����ڵ��������������ܽ���λ��: " << new_filtered_frame_3d_points.size() << std::endl;
		return 502;
	}

	res_err = computeRigidTransformSVD(new_corres_template_points,
		                               new_filtered_frame_3d_points,
		                               Rt);
	if (res_err != 0 || Rt.empty())
	{
		return res_err;
	}

	filtered_frame_3d_points = new_filtered_frame_3d_points;
	corres_template_pnts     = new_corres_template_points;
	//corres_template_points_ID = new_corres_template_points_ID;

	Rt_global = Rt * scan_to_marker_RT;

	LB_COUT << "��ǵ㣺 " << std::endl;
	for (int i = 0; i < filtered_frame_3d_points.size(); i++)
	{
		LB_COUT << filtered_frame_3d_points[i].x << " "<<
			         filtered_frame_3d_points[i].y << " "<<
					 filtered_frame_3d_points[i].z << std::endl;
	}
	LB_COUT << std::endl;

	LB_COUT << "ģ��㣺 " << std::endl;
	for (int i = 0; i < filtered_frame_3d_points.size(); i++)
	{
		LB_COUT << corres_template_pnts[i].x << " " <<
			         corres_template_pnts[i].y << " "<<
					 corres_template_pnts[i].z << std::endl;
	}
	LB_COUT << std::endl;

	LB_COUT << " Opt Rt��from Template to Vision�� is: " << std::endl;
	LB_COUT << std::fixed << std::setprecision(8);  // ǿ�Ʊ��� 8 λС��
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			LB_COUT << std::setw(8) << Rt.at<double>(i, j) << " ";
		}
		LB_COUT << std::endl;
	}

	LB_COUT << " scan_to_marker_RT is: " << std::endl;
	LB_COUT << std::fixed << std::setprecision(8);  // ǿ�Ʊ��� 8 λС��
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			LB_COUT << std::setw(8) << scan_to_marker_RT.at<double>(i, j) << " ";
		}
		LB_COUT << std::endl;
	}

	LB_COUT << " Realtime Rt_global��from Scanner to Vision�� is: " << std::endl;
	LB_COUT << std::fixed << std::setprecision(8);  // ǿ�Ʊ��� 8 λС��
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			LB_COUT << std::setw(8) << Rt_global.at<double>(i, j) << " ";
		}
		LB_COUT << std::endl;
	}
	LB_COUT << "���ζ�λ�Ķ�Ӧ�������� " << new_filtered_frame_3d_points.size() << std::endl;


	//// 9. ������
	//LB_COUT << "���ɹ��ؽ���ά��ǵ�����: " << frame_3d_points.size() << std::endl;
	//for (size_t i = 0; i < frame_3d_points.size(); ++i)
	//{
	//	LB_COUT << frame_3d_points[i].x << "," << frame_3d_points[i].y << "," << frame_3d_points[i].z << std::endl;
	//}
	//int aa = 0;
	return 0;
}
