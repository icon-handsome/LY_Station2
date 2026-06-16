#pragma once
#include "AppConfig.h"
#include <opencv2/opencv.hpp>
#include <vector>

// ������ǵ�ĸ�����Ϣ
struct TrackedPoint 
{
	cv::Point2f pos2d;                          // ��ǰ����������
	bool        is_lost;                        // �����Ƿ�ʧ
	int         consecutive_frames;             // ��������֡��

	TrackedPoint() : pos2d(0, 0), is_lost(true), consecutive_frames(0) {}
};

class MarkPointDetector
{
public:
	// ��������
	struct Config
	{
		int   pyramid_levels;                    // ����������
		int   min_area;                          // ʶ���ǵ����С���
		int   max_area;                          // ʶ���ǵ����С���
		int   ROI_w;                             // �ֲ�����ROI
		int   ROI_h;
		double perimeter_radius_px;
		double min_circularity;
		int   intensity_threshold;
		int   debscan_min_pts;

		Config()                                 // init from AppConfig
		{
			const AppConfig::Detector& detector = AppConfig::Instance().detector;
			pyramid_levels      = detector.pyramid_levels;
			min_area            = detector.min_area;
			max_area            = detector.max_area;
			ROI_w               = detector.ROI_w;
			ROI_h               = detector.ROI_h;
			perimeter_radius_px = detector.perimeter_radius_px;
			min_circularity     = detector.min_circularity;
			intensity_threshold = detector.intensity_threshold;
			debscan_min_pts     = detector.debscan_min_pts;
		}
	} config;

	cv::Rect    roi;                             // Ԥ�����������
	std::vector<TrackedPoint> tracked_points;    // ���ٱ�ǵ�����
	bool is_initialized      = false;            // ���ٵı�ǵ��Ƿ񱻳�ʼ��

	MarkPointDetector() 
	{
		tracked_points.reserve(AppConfig::Instance().limits.mark_point_size_max);
	}
 
	/**************************************************************************************
	*��  �ܣ���ǵ�ͼ��Ԥ�����������
	*��  ����
	*       img_in                      I         ����ĸ߷ֱ���ͼ
	*       results                     O         ����ı�ǵ�Բ������������
	*����ֵ��״̬��
	*��  ע���ú���������չ��ȫ����ֲ������л��ĺ���
	**************************************************************************************/
	bool ProcessFrame(const cv::Mat& img_in, 
		              std::vector<cv::Point2f>& results);

private:
	/**************************************************************************************
	*��  �ܣ�ͼ��������ɴֵ��� (���ڳ�ʼ������ٶ�ʧ)
	*��  ����
	*       img_in                      I         ����ĸ߷ֱ���ͼ
	*       results                     O         ����ı�ǵ�Բ������������
	*����ֵ��״̬��
	*��  ע��
	**************************************************************************************/
	bool GlobalSearch(const cv::Mat& img_in, std::vector<cv::Point2f>& results);
	 
	cv::Point2f RefineSubpixel(const cv::Mat &img,
	                                              cv::Point2f   approx_pos);

	float GetSubpixelGray(const cv::Mat& img, float x, float y);
	 
	cv::Point2f RefineCenter(const cv::Mat& img, cv::Point2f approx_pos, float radius,
		                     cv::Mat K = cv::Mat(), cv::Mat distCoeffs = cv::Mat());

	double calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2);
    
    cv::Point2f calculateCentroid(const std::vector<cv::Point2f>& points);
    
    // ������Ⱥ��
    // ����˵����
    //   points: �����ԭʼ�㼯
    //   std_factor: ��׼��ϵ��������2��3��ֵԽ�����Խ���ɣ�
    // ����ֵ�����˺�ĵ㼯
    // �Ż���
    // 1)����ʽ���ȹ��˵����Ե���Ⱥ�㣬�����¼������ĺ���ֵ���ظ� 1-2 �Σ������ʼ��Ⱥ��Ӱ�����ļ���
    // 2)ԭʼ�㷨����㼯Χ�Ƽ���������̬�ֲ�����ʵ�ʳ����п��ܲ���������������Ż������÷�λ�������ķ�λ�� IQR��
    // ���ó������㼯�ֲ�����̬��������ȷֲ���ƫ̬�ֲ���
    // �����߼���
    // �������о�����ķ�λ�� Q1��25%����Q3��75%��
    // �����ķ�λ�� IQR = Q3 - Q1
    // ��ֵ = Q3 + 1.5 * IQR������ IQR ��Ⱥ���ж�����
    int filterOutliers(const std::vector<cv::Point2f> &points,
    	               std::vector<cv::Point2f>       &filtered_points,
    	               double                         std_factor = 2.0);
    
    
    // ���ǵ���ǵ�ľۼ��ԣ�ʹ��descan�����˲�
    /**
    * DBSCAN ���ಢ�������ص����ĵ�
    * @param points    ����ĵ㼯��cv::Point2f ��ʽ��
    * @param eps       ����뾶�������㱻��Ϊ�ھӵ�������ؾ��룩
    * @param minPts    ���ٵ��������ڴ������ĵ㽫����Ϊ������
    * @return          �Ƿ�ɹ��ҵ���Ч�ľۼ���
    */
    bool filterOutlies_Debscan(const std::vector<cv::Point2f> &points,
    	                      std::vector<cv::Point2f>       &filtered_points,
    						  float eps,
    						  int minPts);
};

// ɨ��ͷ��ǵ�ģ��ƥ��
// ��ϣ����Ŀ���洢�Ƕ�����ֵ�����ĵ�A������
struct Entry
{
	float cosA;               // �����Ӿ͵�ʹ�ã�L1,L2,cosA��,����ά���ڽϴ��L_BINS�£���ϣͰ�����޴��ڴ汬��
	uint8_t pointIdA;
};

// ���ι�ϣ����ά������ұ���ɨ�����ϱ�ǵ���Ծ���ͽǶȲ���
// ��������ѯ��ͶƱ��ʶ��
class FastGeoHash
{
public:
	float               minDistance;   // ģ�����С������ֵ(���ڸ�ֵ�Ĳ�������������Ͳ�ѯ)
	float               maxDistance;   // ģ�����������ֵ(���ڸ�ֵ�Ĳ�������������Ͳ�ѯ)
	float               minDistanceSq; // Ԥ����ƽ��ֵ
	
	float cosTolerance;                 // ���Ծ���Ƕ�Լ��
	float minPercent;
	cv::Mat scan_to_marker_RT;        // ɨ���ǵ�ɨ��ͷ��ǵ�任

	// ����L1��L2������������άդ���ٽ��cosAȷ������ά�ȷ����ϵ��λ��
	// maxDist-ɨ��ͷ�ϱ�ǵ�������
	// minDist-�����������ĵ㲻������������Ͳ�ѯ
	FastGeoHash(float maxDist, float minDist = 60.0f) : maxDistance(maxDist), minDistance(minDist)
	{
		cosTolerance = AppConfig::Instance().geo_hash.cos_tolerance;
		minPercent   = AppConfig::Instance().geo_hash.min_percent;
		minDistanceSq = minDist * minDist;           // Ԥ����
		step          = maxDist / L_BINS;            // maxDistΪ400mmʱ��step==0.1mm��4000x4000 = 16,000,000 ��int��Լ 64MB
		counts        = new int[L_BINS * L_BINS]();
		offsets       = new int[L_BINS * L_BINS]();

		Rt_global = (cv::Mat_<double>(4, 4) << 1.0, 0.0, 0.0, 0.0,
			                                   0.0, 1.0, 0.0, 0.0,
			                                   0.0, 0.0, 1.0, 0.0,
			                                   0.0, 0.0, 0.0, 1.0);

		scan_to_marker_RT = (cv::Mat_<double>(4, 4) << 1.0, 0.0, 0.0, 0.0,
			                                           0.0, 1.0, 0.0, 0.0,
			                                           0.0, 0.0, 1.0, 0.0,
			                                           0.0, 0.0, 0.0, 1.0);
	}

	~FastGeoHash()
	{
		delete[] counts;
		delete[] offsets;
	}

	std::vector<cv::Point3f> template_pnts;             // ģ���
	std::vector<cv::Point3f> filtered_frame_3d_points;  // �˲����3D��ǵ�
	std::vector<int>         corres_template_points_ID; // �˲����Ӧ��ģ��3D��ID
	cv::Mat                  Rt_global;                 // ȫ������ϵ�µ���λ��

	// ���ò���
	int set_template_config(float   minDistance_t,
		                    float   maxDistance_t);

	// ���ò�ѯ����
	int set_query_config(float   cosTolerance_t,
                         float   minPercent_t);

	// ����ɨ���ǵ�ɨ��ͷ��ǵ�ı궨���
	int set_scan_to_marker_RT(cv::Mat &scan_to_marker_RT_t);

	// �õ�ģ���
	int read_template_pnts(const char *file_name);

	// ���߽���
	// ����130��ģ�͵㣬������ϣ��
	int build();

	// �Ƚ�ʶ���
	// sA                Ŀ���
	// otherCandidates  �����е�������ά�ؽ���
	// cosTolerance     �Ƕ��ݲ�
	// minPercent       ���Ʊ��ռ�ȣ�ռ���в�ѯ�����İٷֱ�
	// ���ز��ҵ���id
	int query(const cv::Point3f& sA,
		      const std::vector<cv::Point3f>& otherCandidates,
		      float cosTolerance,
			  float minPercent);

	// ������Ƶı任����Rt
	int computeRigidTransformSVD(const std::vector<cv::Point3f>& src, 
	                         const std::vector<cv::Point3f>& dst, 
							 cv::Mat &Rt);

	// ����λ�ø��ٺ����ӿ�
	// frame_3d_points         ������ĵ��ƣ�˫Ŀ���ٵı�ǵ���ƣ�
	int Get_Track_Pose(std::vector<cv::Point3f>& frame_3d_points,
	                   float cosTolerance,
                       float minPercent);

private:
	static const int    L_BINS = 1300;  // 400mm / 0.1mm = 4000��    650/0.5=1300
	float               step;          // ÿ��Ͱ�ĳ���

	// ���ǵ���ͳ��ϣ����ѯ�У��ײ��ڴ���Ҫָ����ת
	// ��Ҫ��CSR(Compressed Sparse Row) �ṹ���٣�counts/offsets/entries
	int                *counts;
	int                *offsets;
	std::vector<Entry>  entries;

	// ˽��ͶƱ��
	std::vector<int> votes;

	// ����������������ӳ�䵽����
	inline int getIdx(float len) const;

	// ��������������3D�����������Ⱥ�����ֵ��
	// ע�⣺����ֱ�ӷ��س��ȣ���Ϊդ���ǻ��ڳ���mm���ֵ�
	inline bool calcFeature(const cv::Point3f &A,
		                    const cv::Point3f &B,
							const cv::Point3f &C,
		                    float& lAB, 
							float& lAC, 
							float& cosA) const;

	// --- �ڶ����֣����߲�ѯ ---
	// ���볡���е������� A, B, C�����������ν���ͶƱ
	int addVote(const cv::Point3f &sA,
		        const cv::Point3f &sB,
			    const cv::Point3f &sC,
			    float             cosTolerance,
				int               *valid_count);
	 

	void clearVotes();

	// count - ��Ʊ��
	// minPercent - ѡ��id����Сռ��
	int getResult(int count, float minPercent);
};