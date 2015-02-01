
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
		// �E�[�̗L���s�N�Z������������o�^
		do {
			if (check(pImageLine[x])) {
				q[pos++] = x + y;
				break;
			}
		} while (--x >= lx);
		// ���ɑ����L���s�N�Z����skip
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

struct Record {
	uint16_t py;	// parent y
	uint16_t cy;	// current y
	uint16_t cl;	// current left
	uint16_t cr;	// current right
};

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
	uint32_t py = pt.y;
	auto* pImageLine = &pImage[py * imageLineStride];
	if (!check(pImageLine[px])) {
		return;
	}
	auto* pFlagsLine = &pFlags[py * flagsLineStride];
	Record stack[64];
	Record* pStackTop = stack;
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

		if (py - 1 >= limitRange.minY) {
			*pStackTop++ = {
				py,
				py - 1,
				pl,
				pr,
			};
		}
		if (py + 1 <= limitRange.maxY) {
			*pStackTop++ = {
				py,
				py + 1,
				pl,
				pr,
			};
		}
	}
	while (pStackTop > stack) {
		--pStackTop;
		int cy = pStackTop->cy;
		pImageLine = &pImage[cy * imageLineStride];
		pFlagsLine = &pFlags[cy * flagsLineStride];

		Record rec = *pStackTop;
		int py = rec.py;
		int cl = rec.cl;
		int cr = rec.cr;
		// ���ɋL�^�ς݂Ȃ�I��
		if (pFlagsLine[cl]) {
			continue;
		}
		int ll = limitRange.minX;
		int lr = limitRange.maxX;
		int lx, rx;
		// �����͈͂̍��[�ʒu���L���Ȃ�
		if (check(pImageLine[cl])) {
			// ���[�����̒���
			lx = cl - 1;
			for (; lx>=ll; --lx) {
				if (!check(pImageLine[lx])) {
					break;
				}
			}
			++lx;
			if (cl - lx >= 2) {
				// �e�s�̍����𒲍�
				*pStackTop++ = {
					cy,
					py,
					lx,
					cl - 2,
				};
			}
			rx = cl;
		}else {
			// �����͈͂̍��[�ʒu�͖����������̂ŁA�J�n�ʒu�����[�v�Œ��ׂ�
			// �Ȃ��A�e�s�̍����𒲍����Ȃ����͎���
			for (int x=cl+1; x<=cr; ++x) {
				if (check(pImageLine[x])) {
					// ���ɋL�^�ς݂Ȃ�I��
					if (pFlagsLine[x]) {
						goto Label_Continue;
					}
					// �J�n�ʒu����������
					lx = rx = x;
					goto Label_StartFindRX;
				}
			}
			// �S��������Ȃ��̂ŏI��
			continue;
		}

	Label_StartFindRX:
		int gy = cy + cy - py;
		bool isGyAvailable = (gy >= limitRange.minY && gy <= limitRange.maxY);
		
	Label_FindRX:

		// �A������L���͈͂𒲍�
		for (++rx; rx<cr; ++rx) {
			if (!check(pImageLine[rx])) {
				break;
			}
		}
		for (; rx<=lr; ++rx) {
			if (!check(pImageLine[rx])) {
				break;
			}
		}
		--rx;

		// �L���͈͂̋L�^
		for (int x=lx; x<=rx; ++x) {
			pFlagsLine[x] = 1;
		}

		// �e�s�Ɣ��΂̍s
		if (isGyAvailable) {
			// �e�s�Ɣ��Α��̍s�𒲍�����
			*pStackTop++ = {
				cy,
				gy,
				lx,
				rx,
			};
		}
	
		// �����͈͂̉E�[���Q�ȏ��O�ŏI����Ă����ꍇ��
		if (cr - rx >= 2) {
			// �܂��E�[�܂œ��B���Ă��Ȃ��̂łQ�悩�璲���p��
			int lx2 = rx + 2;
			for (; lx2<=cr; ++lx2) {
				if (check(pImageLine[lx2])) {
					// ���̐�ɍ��[������������A�A������L���͈͂𒲍�
					lx = lx2;
					rx = lx2;
					goto Label_FindRX;
				}
			}
		}else {
			// �����͈͂̉E�[���Q�ȏ��ŏI����Ă����ꍇ��
			if (rx - cr >= 2) {
				// �e�s�̉E���𒲍�����
				*pStackTop++ = {
					cy,
					py,
					cr + 2,
					rx,
				};
			}
		}
	Label_Continue:
		;
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

