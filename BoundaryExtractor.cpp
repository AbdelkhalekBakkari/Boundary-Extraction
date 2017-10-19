#include "BoundaryExtractor.h"
#include <itkOffset.h>
#include <itkImageRegionConstIterator.h>
#include <itkConnectedComponentImageFilter.h>
#include <iostream>

void BoundaryExtractor::handle3DImage(Image3DType::Pointer image, std::vector<Point>& points)
{
	std::cout << "enter" << std::endl;
	auto imageSize = image->GetLargestPossibleRegion().GetSize();
	auto slicesCount = imageSize[2];

	ImageType::IndexType start;
	start.Fill(0);

	ImageType::SizeType size;
	size[0] = imageSize[0];
	size[1] = imageSize[1];

	ImageType::RegionType region(start, size);

	ImageType::Pointer sliceImage = ImageType::New();
	sliceImage->SetRegions(region);
	sliceImage->Allocate();

	Image3DType::SizeType sliceSize;
	sliceSize[0] = imageSize[0];
	sliceSize[1] = imageSize[1];
	sliceSize[2] = 1;

	points.clear();

	for(unsigned int slice=0; slice<slicesCount; slice++)
	{
		Image3DType::IndexType sliceStart;
		sliceStart.Fill(0);
		sliceStart[2] = slice;

		Image3DType::RegionType roi(sliceStart, sliceSize);

		itk::ImageRegionIterator<Image3DType> imageItr(image ,roi);
		itk::ImageRegionIterator<ImageType> sliceItr(sliceImage ,sliceImage->GetLargestPossibleRegion());

		imageItr.GoToBegin();
		sliceItr.GoToBegin();

		while(!sliceItr.IsAtEnd())
		{
			sliceItr.Set(imageItr.Get());
			++imageItr;
			++sliceItr;
		}
		std::cout << "enter2" << std::endl;

		auto contours = GetContours(sliceImage);

		std::cout  << "finis" << std::endl;
		for (int i = 0; i < contours.size(); ++i)
		{
			for (int j = 0; j < contours[i].size(); ++j)
			{
				points.push_back(Point(contours[i][j][0], contours[i][j][1]));
			}
		}
	}

	std::cout  << "finis" << std::endl;
}

std::vector<BoundaryExtractor::ContourType> BoundaryExtractor::GetContours(ImageType::Pointer image)
{
	typedef itk::ConnectedComponentImageFilter <ImageType, ImageType> ConnectedComponentImageFilterType;

	ConnectedComponentImageFilterType::Pointer connectedFilter = ConnectedComponentImageFilterType::New();
	connectedFilter->SetInput(image);
	connectedFilter->SetBackgroundValue(0);
	connectedFilter->Update();

	int objectsCount = connectedFilter->GetObjectCount();

	std::vector<BoundaryExtractor::ContourType> contours;

	ImageType::Pointer connectedComponent = connectedFilter->GetOutput();

	for (int i = 0; i < objectsCount; ++i)
		contours.push_back(MooreTrace(connectedComponent, i + 1));

	return contours;
}

std::vector<itk::Offset<2>> BoundaryExtractor::GetAllOffsets()
{
	// In ITK, Offset<2> is indexed with (row, column), so the following
	// sets up the 8-neighbor indices in counter-clockwise order.
	std::vector<itk::Offset<2>> offsets;

	itk::Offset<2> offset;
	offset[0] = -1; offset[1] = -1;
	offsets.push_back(offset);
	offset[0] = -1; offset[1] = 0;
	offsets.push_back(offset);
	offset[0] = -1; offset[1] = 1;
	offsets.push_back(offset);
	offset[0] = 0; offset[1] = 1;
	offsets.push_back(offset);
	offset[0] = 1; offset[1] = 1;
	offsets.push_back(offset);
	offset[0] = 1; offset[1] = 0;
	offsets.push_back(offset);
	offset[0] = 1; offset[1] = -1;
	offsets.push_back(offset);
	offset[0] = 0; offset[1] = -1;
	offsets.push_back(offset);

	return offsets;
}

std::vector<itk::Offset<2>> BoundaryExtractor::GetOrderedOffsets(itk::Offset<2> firstOffset)
{
	std::vector< itk::Offset<2> > orderedOffsets;

	std::vector< itk::Offset<2> > allOffsets = GetAllOffsets();

	// Find the starting point
	unsigned int startingOffset = 0;
	for (unsigned int i = 0; i < allOffsets.size(); ++i)
	{
		if (allOffsets[i] == firstOffset)
		{
			startingOffset = i;
			break;
		}
	}

	// Add the reamining offsets to the end of the list
	for (unsigned int i = startingOffset + 1; i < allOffsets.size(); ++i)
	{
		orderedOffsets.push_back(allOffsets[i]);
	}

	// Add the reamining offsets from the beginning of the list
	for (unsigned int i = 0; i < startingOffset; ++i)
	{
		orderedOffsets.push_back(allOffsets[i]);
	}

	return orderedOffsets;
}

itk::Index<2> BoundaryExtractor::FindNextPixel(ImageType::Pointer image, itk::Index<2> currentPixel, itk::Offset<2>& backtrack, int label)
{
	// The 'backtrack' input has two uses. First, it is used to know where to start the traversal. Second, it returns the next backtrack position by reference.

	itk::Offset<2> startingOffset = backtrack;

	std::vector< itk::Offset<2> > orderedOffsets = GetOrderedOffsets(startingOffset);
	for (int i = 0; i < orderedOffsets.size(); ++i)
	{
		if (image->GetPixel(currentPixel + orderedOffsets[i]) == label)
		{
			if (i != 0)
			{
				backtrack = (currentPixel + orderedOffsets[i - 1]) - (currentPixel + orderedOffsets[i]);
			}
			else
			{
				backtrack = (currentPixel + startingOffset) - (currentPixel + orderedOffsets[i]);
			}

			return currentPixel + orderedOffsets[i];
		}
	}
	throw std::exception("No next pixel - this means there was a pixel that is not connected to anything!");
}

itk::Index<2> BoundaryExtractor::FindFirstPixel(ImageType::Pointer image, itk::Offset<2>& backtrack, int label)
{
	// Perform a raster scan until a non-zero pixel is reached
	itk::ImageRegionConstIterator<ImageType> imageIterator(image, image->GetLargestPossibleRegion());
	itk::Index<2> previousIndex;
	itk::Index<2> firstPixelIndex;

	while (!imageIterator.IsAtEnd())
	{
		// Get the value of the current pixel
		if (imageIterator.Get() == label)
		{
			firstPixelIndex = imageIterator.GetIndex();
			break;
		}

		previousIndex = imageIterator.GetIndex();
		++imageIterator;
	}

	// Return the backtrack pixel by reference
	backtrack = previousIndex - firstPixelIndex;

	return firstPixelIndex;
}

BoundaryExtractor::ContourType BoundaryExtractor::MooreTrace(ImageType::Pointer image, int label)
{
	std::vector< itk::Index<2> > path;

	itk::Offset<2> backtrack;
	itk::Index<2> firstPixel = FindFirstPixel(image, backtrack, label);

	itk::Index<2> currentPixel = firstPixel;

	do
	{
		path.push_back(currentPixel);
		currentPixel = FindNextPixel(image, currentPixel, backtrack, label);
	} while (currentPixel != firstPixel);

	// Close the loop
	path.push_back(firstPixel);

	return path;
}
