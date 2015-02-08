
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <vector>
#include "timer.h"
#include <intrin.h>

#pragma intrinsic(_BitScanForward)

unsigned long long g_time_fill;

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

// http://d.hatena.ne.jp/yaneurao/20090103
// BSF(BitScanForward)
//  1になっているbitを下位からscanして何bit目で見つかったかを返す。
//  引数に0を渡してはいけない。
int bsf(unsigned int mask)
{
    assert(mask);
#ifdef _MSC_VER
    // VC++の場合
    unsigned long index;
    _BitScanForward(&index, mask);
    // これ引数が0だとサーチ失敗を意味するのだが…。
    // まあ、そういう使い方はしてはならないということで…。
    return (int)index;
#elif defined (__x86_64 || i386)
    int ret;
    __asm__("bsfl %1,%0" : "=r"(ret) : "r"(mask));
    return ret;
#else
    // gccの場合
    return __builtin_ctzl(mask);
#endif
}

int tzcnt(uint32_t mask)
{
#if 1
	return _tzcnt_u32(mask);	// BMI1
#else
	return bsf(mask);
#endif
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
	CheckFunc check,
	__m128i threshold,
	__m256i threshold2
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
	const uint16_t maxMinRange = limitRange.maxY - limitRange.minY;
	do {
		py = *--pStackTop;
		//int dir = (py < 0) ? -1 : +1;
		int dir = +1 | (py >> (sizeof(py) * 8 - 1));
		int cy = abs(py + 1);
		assert(cy == abs(py + 1));
		pFlagsLine = &pFlags[cy * flagsLineStride];
		pImageLine = &pImage[cy * imageLineStride];
		int cl = *--pStackTop;
		int cr = *--pStackTop;
		assert(cl >= 0);
		assert(cr >= 0);
	Label_Do:
		//bool isGyAvailable = (dir > 0) ? (cy <= limitRange.maxY) : (cy >= limitRange.minY);
		bool isGyAvailable = (unsigned)(cy - limitRange.minY) <= maxMinRange;
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
			if (cl - lx >= 2) {
				// 親行の左側を調査
				*pStackTop++ = cl - 2;
				*pStackTop++ = lx;
				*pStackTop++ = -(py + 1);
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
					goto Label_FindRX;
				}
			}
			// 全く見つからないので終了
			continue;
		}

		// 連続する有効範囲を調査
	Label_FindRX:
#if 0
		for (; rx<=limitRange.maxX; ++rx) {
			if (!check(pImageLine[rx])) {
				break;
			}
		}
#else
		// TODO: SIMD版も関数object等で
		{
			int len = (limitRange.maxX - rx) + 1;
			if (len) {
#if 1
				{
					__m256i dat32 = _mm256_lddqu_si256((const __m256i*)(pImageLine + rx));
					dat32 = _mm256_cmpeq_epi8(dat32, _mm256_max_epu8(dat32, threshold2));
					int mask = _mm256_movemask_epi8(dat32);
					int tzc = tzcnt(~mask);
					rx += min(len & 31, tzc);
				}
				const __m256i* pSrc = (const __m256i*)(pImageLine + rx);
				int len32x = len >> 5;
				for (int i=0; i<len32x; ++i) {
					__m256i dat32 = _mm256_lddqu_si256(pSrc++);
					dat32 = _mm256_cmpeq_epi8(dat32, _mm256_max_epu8(dat32, threshold2));
					int mask = _mm256_movemask_epi8(dat32);
					if (mask != -1) {
						int tzc = tzcnt(~mask);
						rx += i * 32 + tzc;
						goto Label_EndRepeatSearch;
					}
				}
				rx += len32x << 5;
#else
				{
					// _mm_lddqu_si128
					__m128i dat16 = _mm_loadu_si128((const __m128i*)(pImageLine + rx));
					dat16 = _mm_cmpeq_epi8(dat16, _mm_max_epu8(dat16, threshold));
					int mask = _mm_movemask_epi8(dat16);
					int tzc = tzcnt(~mask);
					rx += min(len & 15, tzc);
				}
				const __m128i* pSrc = (const __m128i*)(pImageLine + rx);
				int len16x = len >> 4;
				for (int i=0; i<len16x; ++i) {
					__m128i dat16 = _mm_loadu_si128(pSrc++);
					dat16 = _mm_cmpeq_epi8(dat16, _mm_max_epu8(dat16, threshold));
					int mask = _mm_movemask_epi8(dat16);
					if (mask != 65535) {
						int tzc = tzcnt(~mask);
						rx += i * 16 + tzc;
						goto Label_EndRepeatSearch;
					}
				}
				rx += len16x << 4;
#endif
			}
		Label_EndRepeatSearch:
			;
		}
#endif
		--rx;
//		printf("rx %d\n", rx);

		// 有効範囲の記録
		for (int x=lx; x<=rx; ++x) {
			pFlagsLine[x] = 1;
		}
		if (cr - rx < 2) {
			if (rx - cr >= 2) {
				// 調査範囲の右端より２つ以上先で終わっていた場合は親行の右側を調査する
				*pStackTop++ = rx;
				*pStackTop++ = cr + 2;
				*pStackTop++ = -(py + 1);
			}
		}else {
			// 調査範囲の右端より２つ以上手前で終わっていた場合は
			// まだ右端まで到達していないので２つ先から調査継続
			int lx2 = rx + 2;
			for (; lx2<=cr; ++lx2) {
				if (check(pImageLine[lx2])) {
					// 親行と反対側の行を調査する
					*pStackTop++ = rx;
					*pStackTop++ = lx;
					*pStackTop++ = (py + 1);

					// その先に左端が見つかったら、連続する有効範囲を調査
					lx = lx2;
					rx = lx2 + 1;
					goto Label_FindRX;
				}
			}
		}
		// 親行と反対側の行を調査する
		++py;
		cy += dir;
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

	unsigned int id;
	unsigned long long started = __rdtscp(&id);
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
				[=](uint8_t val) -> bool { return val >= i; },
				_mm_set1_epi8(i),
				_mm256_set1_epi8(i)
			);
			//printf("%d\n", i);
		}
	}
	unsigned long long ended = __rdtscp(&id);

	printf("%llu %f\n", ended - started, t.ElapsedSecond());
	printf("%f\n", g_time_fill * 100.0 / (ended - started));
	
	return 0;
}

