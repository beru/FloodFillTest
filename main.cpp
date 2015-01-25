
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

template <typename PixelType, typename CheckFunc>
void FloodFill_ScanLine2(
	const PixelType* pImage, int imageLineStride,
	uint8_t* pFlags, int flagsLineStride,
	Point pt,
	const Range& limitRange,
	Range& filledRange,
	CheckFunc check
	)
{
	struct Position {
		uint16_t left;
		uint16_t right;
		uint16_t y;
	};
	struct Record {
		Position cur;
		Position parent;
	};
	Record q[64];
	size_t pos = 0;

	uint32_t px = pt.x;
	uint32_t py = pt.y;
	auto* pImageLine = &pImage[py * imageLineStride];
	if (!check(pImageLine[px])) {
		return;
	}
	auto* pFlagsLine = &pFlags[py * flagsLineStride];

	// 指定された左右範囲内とその左右を調査、レコード追加
	auto scanline = [&](int y, int pl, int pr, int py, bool extendLeft, bool extendRight) {
		if (y < limitRange.minY || y > limitRange.maxY) {
			return;
		}
		int lx = -1;
		// middle
		for (int x=pl; x<=pr; ++x) {
			if (check(pImageLine[x])) {
				lx = x;
				break;
			}
		}
		if (lx == -1) {
			return;
		}
		pFlagsLine[lx] = 1;
		int rx = lx + 1;
		for (; rx <= pr; ++rx) {
			if (check(pImageLine[rx])) {
				pFlagsLine[rx] = 1;
			}
		}
		--rx;

		// extend left
		if (extendLeft && lx == pl) {
			for (; lx>=limitRange.minX; --lx) {
				if (!check(pImageLine[lx])) {
					break;
				}
			}
			++lx;
			for (int x=lx; x<pl; ++x) {
				pFlagsLine[x] = 1;
			}
		}
		// extend right
		if (extendRight && rx == pr) {
			for (; rx<=limitRange.maxX; ++rx) {
				if (!check(pImageLine[rx])) {
					break;
				}
			}
			--rx;
			for (int x=pr+1; x<=rx; ++x) {
				pFlagsLine[x] = 1;
			}
		}

		q[pos++] = {
			{ lx, rx, y, },
			{ pl, pr, py, },
		};
	};

	{
		// first line
		// left
		int pl = px - 1;
		for (; pl>=limitRange.minX; --pl) {
			if (!check(pImageLine[pl])) {
				break;
			}
		}
		++pl;
		// right
		int pr = px + 1;
		for (; pr<=limitRange.maxX; ++pr) {
			if (!check(pImageLine[pr])) {
				break;
			}
		}
		--pr;

		// fill middle
		for (int x=pl; x<=pr; ++x) {
			pFlagsLine[x] = 1;
		}

		pFlagsLine -= flagsLineStride;
		pImageLine -= imageLineStride;
		scanline(py - 1, pl, pr, py, true, true);
		pFlagsLine += 2 * flagsLineStride;
		pImageLine += 2 * imageLineStride;
		scanline(py + 1, pl, pr, py, true, true);
	}
	while (pos--) {
		const Record& r = q[pos];

		Position pp;
		// 親と同じ方向のラインを調査
		pp = r.parent;
		py = pp.y;
		pFlagsLine = &pFlags[py * flagsLineStride];
		pImageLine = &pImage[py * imageLineStride];
		int pl = pp.left;
		int pr = pp.right;
		pp = r.cur;
		int cy = pp.y;
		int cl = pp.left;
		int cr = pp.right;
		if (cl < pl - 1) {
			scanline(py, cl, pl - 2, cy, true, false);
		}
		if (cr > pr + 1) {
			scanline(py, pr + 2, cr, cy, false, true);
		}
		// 親と反対の方向のラインを調査
		int diff = cy - py;
		int y = cy + diff;
		pFlagsLine += diff * 2 * flagsLineStride;
		pImageLine += diff * 2 * imageLineStride;
		scanline(y, cl, cr, cy, true, true);
	}

}

int main(int argc, char* argv[])
{
	const size_t WIDTH = 640;
	const size_t HEIGHT = 480;
	std::vector<uint8_t> src(WIDTH * HEIGHT);
	uint8_t* pSrc = &src[0];
	
	FILE* f = fopen("test.raw", "rb");
	fread(pSrc, 1, WIDTH*HEIGHT, f);
	fclose(f);
	
	Range limitRange = {
		10, WIDTH-10,
		10, HEIGHT-10,
	};
	Range filledRange;

	std::vector<uint8_t> flags(WIDTH * HEIGHT);
	uint8_t* pFlags = &flags[0];

	Timer t;
	t.Start();
	for (size_t i=0; i<128; ++i) {
		
		memset(pFlags, 0, WIDTH * HEIGHT);
//		FloodFill(
//		FloodFill_ScanLine(
		FloodFill_ScanLine2(
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

