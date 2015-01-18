
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <vector>

struct Range {
	uint16_t minX;
	uint16_t maxX;
	uint16_t minY;
	uint16_t maxY;
};

struct Point {
	uint16_t x;
	uint16_t y;
};

template <typename PixelType, typename CheckFunc>
void FloodFill(
	const PixelType* pImage, int imageLineStride,
	uint8_t* pFlags, int flagsLineStride,
	Point pt,
	const Range& limitRange,
	Range& filledRange,
	CheckFunc check
	)
{
	std::vector<Point> q(1024*128);
	size_t pos = 1;
	q[0] = pt;
	while (pos--) {
		auto xy = q[pos];
		auto x = xy.x;
		auto y = xy.y;
		auto pixel = pImage[y * imageLineStride + x];
		auto& flag = &pFlags[y * flagsLineStride + x];
		if (!flag && check(pixel)) {
			flag = 1;
			q[pos++] = {x - 1, y};
			q[pos++] = {x + 1, y};
			q[pos++] = {x, y - 1};
			q[pos++] = {x, y + 1};
		}
	}
}

template <typename PixelType, typename CheckFunc>
void FloodFill_ScanLine(
	const PixelType* pImage, int imageLineStride,
	uint8_t* pFlags, int flagsLineStride,
	Point pt,
	const Range& limitRange,
	Range& filledRange,
	CheckFunc check
	)
{
	std::vector<Point> q(1024);
	size_t pos = 1;
	q[0] = pt;
	while (pos--) {
		pt = q[pos];
		auto* pImageLine = &pImage[pt.y * imageLineStride];
		auto* pFlagsLine = &pFlags[pt.y * flagsLineStride];
		auto& flag = pFlagsLine[pt.x];
		if (!flag && check(pImageLine[pt.x])) {
			flag = 1;
			// left
			int lx = pt.x - 1;
			for (; lx>=limitRange.minX; --lx) {
				if (check(pImageLine[lx])) {
					pFlagsLine[lx] = 1;
				}else {
					break;
				}
			}
			++lx;
			// right
			auto rx = pt.x + 1;
			for (; rx<=limitRange.maxX; ++rx) {
				if (check(pImageLine[rx])) {
					pFlagsLine[rx] = 1;
				}else {
					break;
				}
			}
			--rx;

			auto scanline = [&](int y) {
				auto x = rx;
				while (x >= lx) {
					// 右端の有効ピクセルを見つけたら登録
					for (; x >= lx; --x) {
						if (check(pImageLine[x]) && !pFlagsLine[x]) {
							q[pos++] = {x, y};
							break;
						}
					}
					// 左に続く有効ピクセルはskip
					for (; x >= lx; --x) {
						if (!check(pImageLine[x])) {
							break;
						}
					}
				}
			};
			// up
			pImageLine -= imageLineStride;
			pFlagsLine -= flagsLineStride;
			scanline(pt.y - 1);

			// down
			pImageLine += 2 * imageLineStride;
			pFlagsLine += 2 * flagsLineStride;
			scanline(pt.y + 1);
		}
	}
}

int main(int argc, char* argv[])
{
	const size_t WIDTH = 640;
	const size_t HEIGHT = 480;
	std::vector<uint8_t> src(WIDTH*HEIGHT);
	uint8_t* pSrc = &src[0];
	
	FILE* f = fopen("test.raw", "rb");
	fread(pSrc, 1, WIDTH*HEIGHT, f);
	fclose(f);
	
	std::vector<uint8_t> flags(WIDTH*HEIGHT);
	uint8_t* pFlags = &flags[0];

	Range limitRange = {
		10, WIDTH-10,
		10, HEIGHT-10,
	};
	Range filledRange;

//	FloodFill(
	FloodFill_ScanLine(
		pSrc, WIDTH,
		pFlags, WIDTH,
		{WIDTH/2, HEIGHT/2},
		limitRange,
		filledRange,
		[](uint8_t val) -> bool { return val >= 170; }
	);
	
	return 0;
}

