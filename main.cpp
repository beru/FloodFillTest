
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <vector>
#include "timer.h"

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
		auto& flag = pFlags[y * flagsLineStride + x];
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
	std::vector<uint32_t> q(1024);
	size_t pos = 0;
	q[0] = (pt.y << 16) + pt.x;
	do {
		uint32_t xy = q[pos];
		uint32_t px = xy & 0xFFFF;
		uint32_t py = xy >> 16;
		auto* pFlagsLine = &pFlags[py * flagsLineStride];
		if (pFlagsLine[px]) {
			continue;
		}
		auto* pImageLine = &pImage[py * imageLineStride];
		if (check(pImageLine[px])) {
			// left
			int lx = px - 1;
			for (; lx>=limitRange.minX; --lx) {
				if (!check(pImageLine[lx])) {
					break;
				}
			}
			++lx;
			// right
			auto rx = px + 1;
			for (; rx<=limitRange.maxX; ++rx) {
				if (!check(pImageLine[rx])) {
					break;
				}
			}
			--rx;

			for (auto x=lx; x<=rx; ++x) {
				pFlagsLine[x] = 1;
			}

			auto scanline = [&](int y) {
				int x = rx;
				do {
					// 右端の有効ピクセルを見つけたら登録
					do {
						if (check(pImageLine[x])) {
							q[pos++] = x + y;
							break;
						}
					} while (--x >= lx);
					// 左に続く有効ピクセルはskip
					for (; x >= lx; --x) {
						if (!check(pImageLine[x])) {
							break;
						}
					}
				} while (x >= lx);
			};
			// up
			pImageLine -= imageLineStride;
			pFlagsLine -= flagsLineStride;
			scanline((py - 1) << 16);

			// down
			pImageLine += 2 * imageLineStride;
			pFlagsLine += 2 * flagsLineStride;
			scanline((py + 1) << 16);
		}
	}while (pos--);
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
	
	Range limitRange = {
		10, WIDTH-10,
		10, HEIGHT-10,
	};
	Range filledRange;

	std::vector<uint8_t> flags(WIDTH*HEIGHT);
	uint8_t* pFlags = &flags[0];

	Timer t;
	t.Start();
	for (size_t i=0; i<128; ++i) {
		
		memset(pFlags, 0, WIDTH*HEIGHT);
	//	FloodFill(
		FloodFill_ScanLine(
			pSrc, WIDTH,
			pFlags, WIDTH,
			{WIDTH/2, HEIGHT/2},
			limitRange,
			filledRange,
			[](uint8_t val) -> bool { return val >= 170; }
		);
	}

	printf("%f\n", t.ElapsedSecond());
	printf("%p\n", pFlags);
	
	return 0;
}

