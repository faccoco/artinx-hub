#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


struct ArmorData {
	cv::Point2f armorCenter;
	cv::Point2f energyCenter;
	float angle;
	int quadrant;
	bool isFind;
	ArmorData() {
		armorCenter = cv::Point2f(0, 0);
		energyCenter = cv::Point2f(0, 0);
		angle = 0;
		quadrant = 0;
		isFind = 0;
	}
};

static ArmorData lastData;
bool direction;
bool dirTested;
bool velTested;


bool circleLeastFit(const std::vector<cv::Point2f>& points, cv::Point2f& energyCenter) {
	float center_x = 0.0f;
	float center_y = 0.0f;
	float radius = 0.0f;

	if (points.size() < 3) {
		return false;
	}
	double sum_x = 0.0f, sum_y = 0.0f;
	double sum_x2 = 0.0f, sum_y2 = 0.0f;
	double sum_x3 = 0.0f, sum_y3 = 0.0f;
	double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;
	int N = points.size();
	for (int i = 0; i < N; i++) {
		double x = points[i].x;
		double y = points[i].y;
		double x2 = x * x;
		double y2 = y * y;
		sum_x += x;
		sum_y += y;
		sum_x2 += x2;
		sum_y2 += y2;
		sum_x3 += x2 * x;
		sum_y3 += y2 * y;
		sum_xy += x * y;
		sum_x1y2 += x * y2;
		sum_x2y1 += x2 * y;
	}
	double C, D, E, G, H;
	double a, b, c;
	C = N * sum_x2 - sum_x * sum_x;
	D = N * sum_xy - sum_x * sum_y;
	E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
	G = N * sum_y2 - sum_y * sum_y;
	H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
	a = (H * D - E * G) / (C * G - D * D);
	b = (H * C - E * D) / (D * D - G * C);
	c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;
	center_x = a / (-2);
	center_y = b / (-2);
	radius = sqrt(a * a + b * b - 4 * c) / 2;
	energyCenter = cv::Point2f(center_x, center_y);
	return true;
}

bool setBinary(const cv::Mat src, cv::Mat& binary, const int colorMode)
{
	std::vector<cv::Mat> imgChannels;
	split(binary, imgChannels);

	if (colorMode == 0)
	{
		const auto energeRed = imgChannels[2] - imgChannels[0];
		threshold(energeRed, binary, 100, 255, cv::THRESH_BINARY);
	}
	else if (colorMode == 1)
	{
		const auto energeBlue = imgChannels[0] - imgChannels[2];
		threshold(energeBlue, binary, 100, 255, cv::THRESH_BINARY);
	}
	else
	{
		CAF_LOG_INFO("Binary failed \n");
		return false;
	}
	return true;
}

double getDistance(const cv::Point2f& a, const cv::Point2f& b)
{
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

bool armorJudge(const std::vector<cv::Point>& contour, const cv::RotatedRect& rotatedRect)
{
	cv::Point2f rectPoints[4];
	rotatedRect.points(rectPoints);
	const auto height = std::min(rotatedRect.size.height, rotatedRect.size.width);
	const auto width = std::max(rotatedRect.size.height, rotatedRect.size.width);
	const auto area = contourArea(contour);
	std::vector<cv::Point2f> rectContour;

	for (int i = 0; i < 4; i++) {
		rectContour.push_back(rectPoints[i]);
	}
	const auto match = matchShapes(contour, rectContour, cv::CONTOURS_MATCH_I1, 0.0);
	if (area > 900 && area < 2000
		&& width / height < 3 && width / height>1
		&& contourArea(contour) / rotatedRect.size.area()>0.8
		&& match < 0.3)
		return true;
}

bool stripJudge(const std::vector<cv::Point>& contour, const cv::RotatedRect& rotatedRect)
{
	cv::Point2f rectPoints[4];
	rotatedRect.points(rectPoints);
	double height = std::min(rotatedRect.size.height, rotatedRect.size.width);
	double width = std::max(rotatedRect.size.height, rotatedRect.size.width);
	double area = contourArea(contour);

	if (height * width > 7500 && height * width < 10000
		&& width / height < 5 && width / height>1
		&& contourArea(contour) / rotatedRect.size.area() < 0.7)
		return true;
}

bool changeAngle(const int quadrant, const float angle, float& tranAngle) {
	if (quadrant == 1) {
		tranAngle = angle;
	}
	else if (quadrant == 2) {
		tranAngle = 90 + 90 - angle;
	}
	else if (quadrant == 3) {
		tranAngle = 180 + angle;
	}
	else if (quadrant == 4) {
		tranAngle = 270 + 90 - angle;
	}
	else {
		CAF_LOG_INFO("Quadrant = 0 \n");
		return false;
	}
	return true;
}

float angleCalculate(ArmorData data1, ArmorData data2)
{
	if (direction)
	{
		if (data1.quadrant == data2.quadrant)
		{
			if (data1.quadrant == 1 || data1.quadrant == 3)
			{
				return data1.angle - data2.angle;
			}
			else return data2.angle - data1.angle;
		}
		else if (data1.quadrant == 1 || data1.quadrant == 3)
		{
			return data1.angle + data2.angle;
		}
		else if (data1.quadrant == 2 || data1.quadrant == 4)
		{
			return 180 - data1.angle - data2.angle;
		}
	}
	else if (!direction) {
		if (data1.quadrant == data2.quadrant)
		{
			if (data1.quadrant == 1 || data1.quadrant == 3)
			{
				return data2.angle - data1.angle;
			}
			else return data1.angle - data2.angle;
		}
		else if (data1.quadrant == 1 || data1.quadrant == 3)
		{
			return 180 - data1.angle - data2.angle;
		}
		else if (data1.quadrant == 2 || data1.quadrant == 4)
		{
			return data1.angle + data2.angle;
		}
	}
}

bool getDirection() {
	const int frames = 20;
	static int times = 0;
	static std::vector<ArmorData> datas;
	int positive = 0;
	int negetive = 0;
	float angles[frames];

	if (times < frames && lastData.isFind) {
		datas.push_back(lastData);
		times++;
		return false;
	}
	else {
		if (int(datas.size()) != frames) {
			times = 0;
			datas.clear();
			return false;
		}
		for (int i = 0; i < frames; ++i) {
			changeAngle(datas[i].quadrant, datas[i].angle, angles[i]);
		}
		for (int j = 1; j < 3; ++j) {
			for (int i = 0; i < frames - j; ++i) {
				if ((angles[i] - angles[i + j]) > 0 || (angles[i] - angles[i + j]) < -300) {
					positive++;
				}
				else if ((angles[i] - angles[i + j]) < 0 || (angles[i] - angles[i + j]) > 300) {
					negetive++;
				}
			}
		}
		if (positive > negetive) {
			direction = true;
			CAF_LOG_INFO("Clockwise \n");
		}
		else if (positive < negetive) {
			direction = false;
			CAF_LOG_INFO("Anticlockwise \n");
		}
		times = 0;
		dirTested = true;
		datas.clear();
		return true;
	}
}

bool getArmorCenter(const cv::Mat src, const int color, ArmorData& data, cv::Point2f offset)
{
	auto binary = src.clone();
	setBinary(src, binary, color);
	auto element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	dilate(binary, binary, element);
	element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
	erode(binary, binary, element);

	std::vector<std::vector<cv::Point> > armorContours;
	std::vector<cv::Vec4i> armorHierarchy;
	findContours(binary, armorContours, armorHierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
	const auto armorContoursSize = armorContours.size();
	if (armorContoursSize == 0) {
		CAF_LOG_INFO("Energy detect failed \n");
		return false;
	}
	std::vector<int> conIndexs;
	for (int i = 0; i < armorContoursSize; ++i)
	{
		if (contourArea(armorContours[i]) > 2500 && armorHierarchy[i][3] == -1)
		{
			if (stripJudge(armorContours[i], minAreaRect(armorContours[i])))
			{
				conIndexs.push_back(i);
			}
		}
	}
	if (conIndexs.size() == 0) {
		CAF_LOG_INFO("Strip detect failed: no strip \n");
		return false;
	}

	int index = NULL;
	float minScore = INT_MAX;

	for (int i = 0; i < conIndexs.size(); ++i)
	{
		const float finalLength = arcLength(armorContours[conIndexs[i]], true);
		const float finalArea = contourArea(armorContours[conIndexs[i]]);
		const float score = finalArea + finalLength * 10;

		if (score < minScore) {
			minScore = score;
			index = conIndexs[i];
		}
	}
	if (index == NULL) {
		CAF_LOG_INFO("Strip detect failed: no strip contour \n");
		return false;
	}

	bool findArmor = false;
	const auto finalRect = boundingRect(armorContours[index]);
	const auto finalROI = binary(finalRect);
	cv::RotatedRect finalSqua;
	float maxArea = 0;
	std::vector<std::vector<cv::Point> > finalContours;
	std::vector<cv::Vec4i> finalHierarchy;

	findContours(finalROI, finalContours, finalHierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, finalRect.tl());
	for (size_t i = 0; i < finalContours.size(); ++i) {
		if (finalHierarchy[i][3] != -1) {
			cv::RotatedRect squa = minAreaRect(finalContours[i]);
			if (armorJudge(finalContours[i], squa))
			{
				float area = contourArea(finalContours[i]);
				if (area > maxArea) {
					maxArea = area;
					finalSqua = squa;
					findArmor = true;
				}
			}
		}
	}
	if (findArmor == false) {
		CAF_LOG_INFO("Armor detect failed \n");
		return false;
	}
	data.armorCenter = finalSqua.center + offset;
	const auto finalRrect = minAreaRect(armorContours[index]);
	const auto arrowCenter = finalRrect.center + offset;
	const auto min = MIN(finalSqua.size.height, finalSqua.size.width);

	if (getDistance(arrowCenter, data.armorCenter) < min * 0.8) {
		data.isFind = false;
	}
	else {
		float tranAngle = 0.0;
		if (finalSqua.size.width > finalSqua.size.height) {
			tranAngle = 90 - fabs(finalSqua.angle);
		}
		else {
			tranAngle = fabs(finalSqua.angle);
		}
		data.angle = tranAngle;
		if (tranAngle < 20) {
			if (finalSqua.size.width > finalSqua.size.height
				&& arrowCenter.x < data.armorCenter.x) {
				data.quadrant = 1;
			}
			else if (finalSqua.size.width < finalSqua.size.height
				&& arrowCenter.x > data.armorCenter.x) {
				data.quadrant = 2;
			}
			else if (finalSqua.size.width > finalSqua.size.height
				&& arrowCenter.x > data.armorCenter.x) {
				data.quadrant = 3;
			}
			else if (finalSqua.size.width < finalSqua.size.height
				&& arrowCenter.x < data.armorCenter.x) {
				data.quadrant = 4;
			}
		}
		else if (tranAngle > 70) {
			if (finalSqua.size.width > finalSqua.size.height
				&& arrowCenter.y > data.armorCenter.y) {
				data.quadrant = 1;
			}
			else if (finalSqua.size.width < finalSqua.size.height
				&& arrowCenter.y > data.armorCenter.y) {
				data.quadrant = 2;
			}
			else if (finalSqua.size.width > finalSqua.size.height
				&& arrowCenter.y < data.armorCenter.y) {
				data.quadrant = 3;
			}
			else if (finalSqua.size.width < finalSqua.size.height
				&& arrowCenter.y < data.armorCenter.y) {
				data.quadrant = 4;
			}
		}
		else {
			if (arrowCenter.x < data.armorCenter.x && arrowCenter.y >= data.armorCenter.y
				&& finalSqua.size.width > finalSqua.size.height) {
				data.quadrant = 1;
			}
			else if (arrowCenter.x >= data.armorCenter.x && arrowCenter.y > data.armorCenter.y
				&& finalSqua.size.width <= finalSqua.size.height) {
				data.quadrant = 2;
			}
			else if (arrowCenter.x > data.armorCenter.x && arrowCenter.y <= data.armorCenter.y
				&& finalSqua.size.width > finalSqua.size.height) {
				data.quadrant = 3;
			}
			else if (arrowCenter.x <= data.armorCenter.x && arrowCenter.y < data.armorCenter.y
				&& finalSqua.size.width <= finalSqua.size.height) {
				data.quadrant = 4;
			}
		}
		if (data.quadrant == 1) {
			data.energyCenter.x = data.armorCenter.x - 148 * cos(data.angle * CV_PI / 180);
			data.energyCenter.y = data.armorCenter.y + 148 * sin(data.angle * CV_PI / 180);
		}
		else if (data.quadrant == 2) {
			data.energyCenter.x = data.armorCenter.x + 148 * cos(data.angle * CV_PI / 180);
			data.energyCenter.y = data.armorCenter.y + 148 * sin(data.angle * CV_PI / 180);
		}
		else if (data.quadrant == 3) {
			data.energyCenter.x = data.armorCenter.x + 148 * cos(data.angle * CV_PI / 180);
			data.energyCenter.y = data.armorCenter.y - 148 * sin(data.angle * CV_PI / 180);
		}
		else if (data.quadrant == 4) {
			data.energyCenter.x = data.armorCenter.x - 148 * cos(data.angle * CV_PI / 180);
			data.energyCenter.y = data.armorCenter.y - 148 * sin(data.angle * CV_PI / 180);
		}
		data.isFind = true;
	}
}

bool predict(const ArmorData data, cv::Point2f& preCenter, const int predictMode, const int direction) {
	if (predictMode == 0) {
		static int count = 0;
		static std::vector<cv::Point2f> armorPoints;
		armorPoints.resize(50);
		if (count < 50) {
			armorPoints.insert(armorPoints.begin() + count, data.armorCenter);
			count++;
			return false;
		}
		else if (count == 50) {
			cv::Point2f center;
			circleLeastFit(armorPoints, center);
			float preAngle;
			if (direction == 0) {
				preAngle = CV_PI / 7.8;
			}
			else {
				preAngle = -CV_PI / 7.8;
			}
			double x = data.armorCenter.x - center.x;
			double y = data.armorCenter.y - center.y;
			preCenter.x = x * cos(preAngle) + y * sin(preAngle) + center.x;
			preCenter.y = -x * sin(preAngle) + y * cos(preAngle) + center.y;

			return true;
		}
	}
	else if (predictMode == 1) {
		if (data.energyCenter == cv::Point2f(0, 0)) {
			return false;
		}

		float preAngle;
		if (direction == 0) {
			preAngle = CV_PI / 7.8;
		}
		else {
			preAngle = -CV_PI / 7.8;
		}
		const auto x = data.armorCenter.x - data.energyCenter.x;
		const auto y = data.armorCenter.y - data.energyCenter.y;
		preCenter.x = x * cos(preAngle) + y * sin(preAngle) + data.energyCenter.x;
		preCenter.y = -x * sin(preAngle) + y * cos(preAngle) + data.energyCenter.y;

		return true;

	}
	else if (predictMode == 2) {
		const auto preAngle = CV_PI / 7.8;
		const auto dis = 148 * tan(preAngle);
		const auto dis_x = dis * sin(data.angle * CV_PI / 180);
		const auto dis_y = dis * cos(data.angle * CV_PI / 180);

		cv::Point2f tangent;
		if (direction == 0) {

			if (data.quadrant == 1) {
				tangent.x = data.armorCenter.x - dis_x;
				tangent.y = data.armorCenter.y - dis_y;
			}
			else if (data.quadrant == 2) {
				tangent.x = data.armorCenter.x - dis_x;
				tangent.y = data.armorCenter.y + dis_y;
			}
			else if (data.quadrant == 3) {
				tangent.x = data.armorCenter.x + dis_x;
				tangent.y = data.armorCenter.y + dis_y;
			}
			else if (data.quadrant == 4) {
				tangent.x = data.armorCenter.x + dis_x;
				tangent.y = data.armorCenter.y - dis_y;
			}
			else {
				return false;
			}

			const auto x = tangent.x - data.armorCenter.x;
			const auto y = tangent.y - data.armorCenter.y;
			preCenter.x = x * cos(preAngle / 2) + y * sin(preAngle / 2) + data.armorCenter.x;
			preCenter.y = -x * sin(preAngle / 2) + y * cos(preAngle / 2) + data.armorCenter.y;
		}
		else {
			if (data.quadrant == 1) {
				tangent.x = data.armorCenter.x + dis_x;
				tangent.y = data.armorCenter.y + dis_y;
			}
			else if (data.quadrant == 2) {
				tangent.x = data.armorCenter.x + dis_x;
				tangent.y = data.armorCenter.y - dis_y;
			}
			else if (data.quadrant == 3) {
				tangent.x = data.armorCenter.x - dis_x;
				tangent.y = data.armorCenter.y - dis_y;
			}
			else if (data.quadrant == 4) {
				tangent.x = data.armorCenter.x - dis_x;
				tangent.y = data.armorCenter.y + dis_y;
			}
			else {
				return false;
			}

			const auto x = tangent.x - data.armorCenter.x;
			const auto y = tangent.y - data.armorCenter.y;
			preCenter.x = x * cos(-preAngle / 2) + y * sin(-preAngle / 2) + data.armorCenter.x;
			preCenter.y = -x * sin(-preAngle / 2) + y * cos(-preAngle / 2) + data.armorCenter.y;
		}
		return true;
	}
}

void detect(const cv::Mat frame, const int color, const int Mode, cv::Point2f& predictPoint) {
	cv::Point2f offset = cv::Point2f(0, 0);

	if (Mode == 0) {
		ArmorData armordata;
		if (getArmorCenter(frame, color, armordata, offset) == false) {
			predictPoint = cv::Point2f(0, 0);
		}
		else if (dirTested) {
			predict(armordata, predictPoint, 1, direction);
		}
		lastData = armordata;
	}
	else if (Mode == 1) {
		ArmorData armordata;
		if (getArmorCenter(frame, color, armordata, offset) == false) {
			predictPoint = cv::Point2f(0, 0);
		}
		else {
			cv::Point2f preCenter;
			if (predict(armordata, preCenter, 2, 0) == false) {
				predictPoint = cv::Point2f(0, 0);
			}
			else {
				predictPoint = preCenter;
			}
		}
		lastData = armordata;
	}
	if (!dirTested)
	{
		getDirection();
	}
}

bool velocityCalculate()
{
	const int frameNums = 50;
	static int times = 0;
	static std::vector<ArmorData> datas;
	float circleAngle[frameNums];
	datas.resize(frameNums);
	if (times < frameNums && lastData.isFind) {
		datas.insert(datas.begin() + times, lastData);
		times++;
		return false;
	}
	else {
		if (int(datas.size()) != frameNums) {
			times = 0;
			datas.clear();
			return false;
		}
		if (times == frameNums)
		{
			for (int i = 0; i < 149; ++i)
			{
				circleAngle[i] = angleCalculate(datas[i], datas[i + 1]);
			}
			velTested = true;
			return false;
		}
		times = 0;
		dirTested = 1;
		datas.clear();
		return true;
	}
}
