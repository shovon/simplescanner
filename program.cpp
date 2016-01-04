#include <iostream>
#include <opencv2/opencv.hpp>
#include <numeric>
#include <vector>

using CornersType = std::vector<std::vector<cv::Point>>;
using ContourType = std::vector<std::vector<cv::Point>>;

ContourType findContours(cv::Mat const &binary_input) {
    ContourType contours;
    cv::findContours(binary_input, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    return contours;
}

// TODO: consider using a vector ADT for the x and y coordinates, and a square
//   ADT for the square.
float computeDistanceFromCenter( int centerX, int centerY, const std::vector<cv::Point>& square ) {
  std::vector<cv::Point2f> centeredSquare;

  // Get the square centered.
  for (size_t i = 0; i < square.size(); i++) {
    cv::Point2f newPoint(square[i].x - centerX, square[i].y - centerY);
    centeredSquare.push_back(newPoint);
  }

  cv::Point2f accum(0, 0);
  for (size_t i = 0; i < centeredSquare.size(); i++) {
    accum.x += centeredSquare[i].x;
    accum.y += centeredSquare[i].y;
  }

  accum.x /= (float) centeredSquare.size();
  accum.y /= (float) centeredSquare.size();

  return std::sqrt(accum.x * accum.x + accum.y * accum.y);
}

// TODO: rename this method.
// TODO: consider generalizing this function to exclusively accept coordinates
//   instead of an image.
std::vector<cv::Point> findCentered( const cv::Mat& image, const std::vector<std::vector<cv::Point>>& squares) {
  const int centerX = image.cols / 2;
  const int centerY = image.rows / 2;
  int closestIndex = 0;
  float closest = std::numeric_limits<float>::max();
  for (size_t i = 0; i < squares.size(); i++) {
    const float distance = computeDistanceFromCenter(centerX, centerY, squares[i]);
    if (distance < closest) {
      closest = distance;
      closestIndex = i;
    }
  }
  return squares[closestIndex];
}

template<typename T, typename U>
void convertPoints(std::vector<T> const &src, std::vector<U> &dst) {
  dst.clear();
  for(auto const &point : src){
      dst.emplace_back(point.x, point.y);
  }
}

template<typename T>
void sortQuadCorners(std::vector<T> &corners)
{
  std::vector<T> top, bot;
  T const center = std::accumulate(
    std::begin(corners),
    std::end(corners),
    cv::Point(0, 0)) * (1. / corners.size()
  );
  for (size_t i = 0; i < corners.size(); i++) {
    if (corners[i].y < center.y) {
      top.emplace_back(corners[i]);
    } else {
      bot.emplace_back(corners[i]);
    }
  }

  T tl = top[0].x > top[1].x ? top[1] : top[0];
  T tr = top[0].x > top[1].x ? top[0] : top[1];
  T bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
  T br = bot[0].x > bot[1].x ? bot[0] : bot[1];

  corners.clear();
  corners.emplace_back(tl);
  corners.emplace_back(tr);
  corners.emplace_back(br);
  corners.emplace_back(bl);
}

cv::Point2f getDimensions(const std::vector<cv::Point> &corners) {
  cv::Point2f tl = corners[0];
  cv::Point2f tr = corners[1];
  cv::Point2f bl = corners[3];
  const float h = cv::norm(tl - tr);
  const float w = cv::norm(tl - bl);
  return cv::Point2f(w, h);
}

void cannyEdgeMethod(cv::Mat image) {
  cv::imshow("The image", image);

  // Step 2: get the edges.

  cv::Mat shrunk;
  cv::resize(image, shrunk, cv::Size(image.cols / 8, image.rows / 8));

  // Step 2.1: get a blurred image.
  cv::Mat blurred = shrunk.clone();
  cv::blur( blurred, blurred, cv::Size(3,3) );

  cv::imshow("blurred", blurred);

  // Step 2.2: get the edges
  cv::Mat edges;
  cv::Canny(blurred, edges, 100, 255);

  cv::imshow("Edges", edges);
  cv::imwrite("edges.jpg", edges);
  cv::waitKey();

  auto contours = findContours(edges);

  cv::Mat contourMap = cv::Mat::zeros(edges.size(), CV_8U);
  cv::drawContours(contourMap, contours, -1, cv::Scalar(255));
  cv::imshow("contour map", contourMap);

  std::vector<std::vector<cv::Point>> quads;
  for (auto contour : contours) {
    std::vector<cv::Point> approx;
    cv::approxPolyDP(cv::Mat(contour), approx, cv::arcLength(cv::Mat(contour), true)*0.02, true);

    if (approx.size() == 4 && cv::isContourConvex(cv::Mat(approx))) {
      quads.push_back(approx);
    }
  }

  auto centered = findCentered(image, quads);

  sortQuadCorners(centered);
  for (auto &point : centered) {
    point *= 8;
  }

  auto destDimensions = getDimensions(centered);
  cv::Mat target(destDimensions.x, destDimensions.y, image.type());
  std::vector<cv::Point2f> targetPoints {
    {0., 0.},
    {(float) (target.cols - 1), 0.},
    {(float) (target.cols - 1), (float) (target.rows - 1)},
    {0., (float) (target.rows - 1)}
  };
  std::vector<cv::Point2f> points;
  convertPoints(centered, points);
  const cv::Mat transMat = cv::getPerspectiveTransform(points, targetPoints);
  cv::warpPerspective(image, target, transMat, target.size());

  cv::imshow("transformed", target);
  cv::imwrite("transformed.jpg", target);

  cv::waitKey();
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cout << "Usage: ./bigsals <image file>" << std::endl;
    return -1;
  }

  auto imageFilename = argv[1];

  // Step 1: get the image.

  cv::Mat image;
  image = cv::imread(imageFilename, 1);

  if (image.empty()) {
    std::cerr << "Unable to open image" << std::endl;
    return -1;
  }

  cannyEdgeMethod(image);

  return 0;
}
