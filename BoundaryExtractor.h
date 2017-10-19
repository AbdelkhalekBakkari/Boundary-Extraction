#ifndef BoundaryExtractor_h__
#define BoundaryExtractor_h__

// ITK
#include <itkImage.h>

// STL
#include <vector>

class Point
{
public:
	Point(float _x, float _y) : x(_x), y(_y), quality(0.f) {}
	float x, y, quality;

	bool operator==(const Point &other) const {
		return (x == other.x && y == other.y);
	}
};

class BoundaryExtractor
{
	typedef itk::Image<short, 2> ImageType;
	typedef itk::Image<short, 3> Image3DType;
	typedef std::vector<itk::Index<2>> ContourType;
public:
	static void handle3DImage(Image3DType::Pointer image, std::vector<Point>& points);
	static std::vector<ContourType> GetContours(ImageType::Pointer image);

private:
	static std::vector<itk::Offset<2>> GetAllOffsets();

	static std::vector<itk::Offset<2>> GetOrderedOffsets(itk::Offset<2> firstOffset);

	static itk::Index<2> FindFirstPixel(ImageType::Pointer image, itk::Offset<2>& backtrack, int label);

	static itk::Index<2> FindNextPixel(ImageType::Pointer image, itk::Index<2> currentPixel, itk::Offset<2>& backtrack, int label);

	static ContourType MooreTrace(ImageType::Pointer image, int label);
};
#endif // BoundaryExtractor_h__
