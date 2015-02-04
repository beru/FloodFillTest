
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
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
__forceinline
void scanline(
	int y,
	int lx,
	int rx,
	uint32_t* q,
	size_t& pos,
	CheckFunc check,
	PixelType* pImageLine
	)
{
	assert(y != -1);
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
	uint32_t stack[1024];
	size_t pos = 0;
	stack[0] = (pt.y << 16) + pt.x;
	do {
		int xy = stack[pos];
		int px = xy & 0xFFFF;
		int py = xy >> 16;
		assert(py >= limitRange.minY);
		assert(py <= limitRange.maxY);
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
			int rx = px + 1;
			for (; rx<=limitRange.maxX; ++rx) {
				if (!check(pImageLine[rx])) {
					break;
				}
			}
			--rx;

			for (auto x=lx; x<=rx; ++x) {
				pFlagsLine[x] = 1;
			}


			int ny = py - 1;
			if (ny >= limitRange.minY) {
				// up
				pImageLine -= imageLineStride;
				pFlagsLine -= flagsLineStride;
				scanline(ny << 16, lx, rx, stack, pos, check, pImageLine);
			}

			ny = py + 1;
			if (ny <= limitRange.maxY) {
				// down
				pImageLine = &pImage[ny * imageLineStride];
				pFlagsLine = &pFlags[ny * flagsLineStride];
				scanline(ny << 16, lx, rx, stack, pos, check, pImageLine);
			}
		}
	}while (pos--);
}

//struct Record {
//	int16_t py;		// parent y, sign bit indicates current line direction
//	uint16_t cl;	// current left
//	uint16_t cr;	// current right
//};

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
	uint32_t px = pt.x;
	int py = pt.y;
	auto* pImageLine = &pImage[py * imageLineStride];
	if (!check(pImageLine[px])) {
		return;
	}
	auto* pFlagsLine = &pFlags[py * flagsLineStride];
	int16_t stack[3 * 256];
	int16_t* pStackTop = stack;
	// first line
	{
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

		*pStackTop++ = pr;
		*pStackTop++ = pl;
		*pStackTop++ = -py;
		*pStackTop++ = pr;
		*pStackTop++ = pl;
		*pStackTop++ = +py;
	}
	do {
		py = *--pStackTop;
		int dir = (py < 0) ? -1 : +1;
		int cy = abs(py + 1);
		pFlagsLine = &pFlags[cy * flagsLineStride];
		pImageLine = &pImage[cy * imageLineStride];
		int cl = *--pStackTop;
		int cr = *--pStackTop;
		assert(cl >= 0);
		assert(cr >= 0);
	Label_Do:
		bool isGyAvailable = (dir > 0) ? (cy <= limitRange.maxY) : (cy >= limitRange.minY);
		// 領域外だったり既に記録済みなら終了
		if (!isGyAvailable || pFlagsLine[cl]) {
			continue;
		}
		int lx, rx;
		// 調査範囲の左端位置が有効なら
		if (check(pImageLine[cl])) {
			// 左端延長の調査
			lx = cl - 1;
			for (; lx>=limitRange.minX; --lx) {
				if (!check(pImageLine[lx])) {
					break;
				}
			}
			++lx;
			cy *= dir;
			if (cl - lx >= 2) {
				// 親行の左側を調査
				*pStackTop++ = cl - 2;
				*pStackTop++ = lx;
				*pStackTop++ = -cy;
			}
			rx = lx + 1;
		}else {
			// 調査範囲の左端位置は無効だったので、開始位置をループで調べる
			// なお、親行の左側を調査しない事は自明
			for (int x=cl+1; x<=cr; ++x) {
				if (check(pImageLine[x])) {
					// 既に記録済みなら終了
					if (pFlagsLine[x]) {
						break;
					}
					// 開始位置が見つかった
					lx = x;
					rx = x + 1;
					cy *= dir;
					goto Label_FindRX;
				}
			}
			// 全く見つからないので終了
			continue;
		}

		// 連続する有効範囲を調査
	Label_FindRX:
		for (; rx<cr; ++rx) {
			if (!check(pImageLine[rx])) {
				break;
			}
		}
	Label_FindRX2:
		for (; rx<=limitRange.maxX; ++rx) {
			if (!check(pImageLine[rx])) {
				break;
			}
		}
		--rx;

		// 有効範囲の記録
		for (int x=lx; x<=rx; ++x) {
			pFlagsLine[x] = 1;
		}

		if (cr - rx < 2) {
			if (rx - cr >= 2) {
				// 調査範囲の右端より２つ以上先で終わっていた場合は親行の右側を調査する
				*pStackTop++ = rx;
				*pStackTop++ = cr + 2;
				*pStackTop++ = -cy;
			}
		}else {
			// 調査範囲の右端より２つ以上手前で終わっていた場合は
			// まだ右端まで到達していないので２つ先から調査継続
			int lx2 = rx + 2;
			for (; lx2<cr; ++lx2) {
				if (check(pImageLine[lx2])) {
					// 親行と反対側の行を調査する
					*pStackTop++ = rx;
					*pStackTop++ = lx;
					*pStackTop++ = cy;

					// その先に左端が見つかったら、連続する有効範囲を調査
					lx = lx2;
					rx = lx2 + 1;
					goto Label_FindRX;
				}
			}
			if (check(pImageLine[lx2])) {
				// 親行と反対側の行を調査する
				*pStackTop++ = rx;
				*pStackTop++ = lx;
				*pStackTop++ = cy;

				// その先に左端が見つかったら、連続する有効範囲を調査
				lx = lx2;
				rx = lx2 + 1;
				goto Label_FindRX2;
			}
		}
		// 親行と反対側の行を調査する
		py = cy;
		cy = abs(cy + 1);
		pFlagsLine += dir * flagsLineStride;
		pImageLine += dir * imageLineStride;
		cl = lx;
		cr = rx;
		goto Label_Do;

	} while (pStackTop > stack);
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
	printf("%p\n", pFlags);
	Timer t;
	t.Start();
	for (size_t nTest=0; nTest<10; ++nTest) {
		for (size_t i=0; i<256; ++i) {
			memset(pFlags, 0, WIDTH * HEIGHT);
			//FloodFill(
			//FloodFill_ScanLine(
			FloodFill_ScanLine2(
				pSrc, WIDTH,
				pFlags, WIDTH,
				{WIDTH/2, HEIGHT/2},
				limitRange,
				filledRange,
				[=](uint8_t val) -> bool { return val >= i; }
			);
			//printf("%d\n", i);
		}
	}

	printf("%f\n", t.ElapsedSecond());
	
	return 0;
}

