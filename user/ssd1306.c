#include"ssd1306.h"

static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];


static SSD1306_t SSD1306;

static void ssd1306_WriteCommand(uint8_t command);

static void ssd1306_WriteCommand(uint8_t command)
{
	HAL_I2C_Mem_Write(&hi2c1, SSD1306_I2C_ADDR, 0x00, 1, &command, 1, 20);
}

uint8_t ssd1306_Init(void)
{	
	HAL_Delay(100);
	
	ssd1306_WriteCommand(0xAE); //display off
	ssd1306_WriteCommand(0x20); //Set Memory Addressing Mode   
	ssd1306_WriteCommand(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	ssd1306_WriteCommand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	ssd1306_WriteCommand(0xC8); //Set COM Output Scan Direction
	ssd1306_WriteCommand(0x00); //---set low column address
	ssd1306_WriteCommand(0x10); //---set high column address
	ssd1306_WriteCommand(0x40); //--set start line address
	ssd1306_WriteCommand(0x81); //--set contrast control register
	ssd1306_WriteCommand(0xFF);
	ssd1306_WriteCommand(0xA1); //--set segment re-map 0 to 127
	ssd1306_WriteCommand(0xA6); //--set normal display
	ssd1306_WriteCommand(0xA8); //--set multiplex ratio(1 to 64)
	ssd1306_WriteCommand(0x3F); //
	ssd1306_WriteCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	ssd1306_WriteCommand(0xD3); //-set display offset
	ssd1306_WriteCommand(0x00); //-not offset
	ssd1306_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
	ssd1306_WriteCommand(0xF0); //--set divide ratio
	ssd1306_WriteCommand(0xD9); //--set pre-charge period
	ssd1306_WriteCommand(0x22); //
	ssd1306_WriteCommand(0xDA); //--set com pins hardware configuration
	ssd1306_WriteCommand(0x12);
	ssd1306_WriteCommand(0xDB); //--set vcomh
	ssd1306_WriteCommand(0x20); //0x20,0.77xVcc
	ssd1306_WriteCommand(0x8D); //--set DC-DC enable
	ssd1306_WriteCommand(0x14); //
	ssd1306_WriteCommand(0xAF); //--turn on SSD1306 panel
	
	ssd1306_Fill(Black);
	
	ssd1306_UpdateScreen();

	SSD1306.CurrentX = 0;
	SSD1306.CurrentY = 0;
	
	SSD1306.Initialized = 1;
	
	return 1;
}

void ssd1306_Fill(SSD1306_COLOR color) 
{
	uint32_t i;

	for(i = 0; i < sizeof(SSD1306_Buffer); i++)
	{
		SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
	}
}

void ssd1306_UpdateScreen(void) 
{
	uint8_t i;
	
	for (i = 0; i < 8; i++) {
		ssd1306_WriteCommand(0xB0 + i);
		ssd1306_WriteCommand(0x00);
		ssd1306_WriteCommand(0x10);
		HAL_I2C_Mem_Write(&hi2c1, SSD1306_I2C_ADDR, 0x40, 1, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH, 150);
	}
}

void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) 
	{
		return;
	}
	
	if (SSD1306.Inverted) 
	{
		color = (SSD1306_COLOR)!color;
	}
	
	if (color == White)
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} 
	else 
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
{
	uint32_t i, b, j;
	
	if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight))
	{
		return 0;
	}
	
	for (i = 0; i < Font.FontHeight; i++)
	{
		b = Font.data[(ch - 32) * Font.FontHeight + i];
		for (j = 0; j < Font.FontWidth; j++)
		{
			if ((b << j) & 0x8000) 
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
			} 
			else 
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
			}
		}
	}
	
	SSD1306.CurrentX += Font.FontWidth;
	
	return ch;
}

char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color)
{
	while (*str) 
	{
		if (ssd1306_WriteChar(*str, Font, color) != *str)
		{
			return *str;
		}
		
		str++;
	}
	
	return *str;
}

void ssd1306_SetCursor(uint8_t x, uint8_t y) 
{
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

void ssd1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR c)
{
	int16_t dx, dy, sx, sy, err, e2, i, tmp; 
	
	if (x0 >= SSD1306_WIDTH) {
		x0 = SSD1306_WIDTH - 1;
	}
	if (x1 >= SSD1306_WIDTH) {
		x1 = SSD1306_WIDTH - 1;
	}
	if (y0 >= SSD1306_HEIGHT) {
		y0 = SSD1306_HEIGHT - 1;
	}
	if (y1 >= SSD1306_HEIGHT) {
		y1 = SSD1306_HEIGHT - 1;
	}
	
	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1); 
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1); 
	sx = (x0 < x1) ? 1 : -1; 
	sy = (y0 < y1) ? 1 : -1; 
	err = ((dx > dy) ? dx : -dy) / 2; 

	if (dx == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}
		
		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}
		
		for (i = y0; i <= y1; i++) {
			ssd1306_DrawPixel(x0, i, c);
		}
		
		return;
	}
	
	if (dy == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}
		
		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}
		
		for (i = x0; i <= x1; i++) {
			ssd1306_DrawPixel(i, y0, c);
		}
		
		return;
	}
	
	while (1) {
		ssd1306_DrawPixel(x0, y0, c); 
		if (x0 == x1 && y0 == y1) {
			break;
		}
		e2 = err; 
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		} 
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		} 
	}
}

void ssd1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR c)
{
	if(
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT ) {
		return;
	}
	
	if ((x + w) >= SSD1306_WIDTH) {
		w = SSD1306_WIDTH - x;
	}
	if ((y + h) >= SSD1306_HEIGHT) {
		h = SSD1306_HEIGHT - y;
	}
	
	ssd1306_DrawLine(x, y, x + w, y, c);         /* Top line */
	ssd1306_DrawLine(x, y + h, x + w, y + h, c); /* Bottom line */
	ssd1306_DrawLine(x, y, x, y + h, c);         /* Left line */
	ssd1306_DrawLine(x + w, y, x + w, y + h, c); /* Right line */
}

void ssd1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR c)
{
	uint8_t i;
	
	if (x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT) {
		return;
	}
	
	if ((x + w) >= SSD1306_WIDTH) {
		w = SSD1306_WIDTH - x;
	}
	if ((y + h) >= SSD1306_HEIGHT) {
		h = SSD1306_HEIGHT - y;
	}
	
	for (i = 0; i <= h; i++) {
		ssd1306_DrawLine(x, y + i, x + w, y + i, c);
	}
}

void ssd1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR color)
{
	ssd1306_DrawLine(x1, y1, x2, y2, color);
	ssd1306_DrawLine(x2, y2, x3, y3, color);
	ssd1306_DrawLine(x3, y3, x1, y1, color);
}


void ssd1306_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR color)
{
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
	curpixel = 0;
	
	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay){
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		ssd1306_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

void ssd1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR c)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    ssd1306_DrawPixel(x0, y0 + r, c);
    ssd1306_DrawPixel(x0, y0 - r, c);
    ssd1306_DrawPixel(x0 + r, y0, c);
    ssd1306_DrawPixel(x0 - r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ssd1306_DrawPixel(x0 + x, y0 + y, c);
        ssd1306_DrawPixel(x0 - x, y0 + y, c);
        ssd1306_DrawPixel(x0 + x, y0 - y, c);
        ssd1306_DrawPixel(x0 - x, y0 - y, c);

        ssd1306_DrawPixel(x0 + y, y0 + x, c);
        ssd1306_DrawPixel(x0 - y, y0 + x, c);
        ssd1306_DrawPixel(x0 + y, y0 - x, c);
        ssd1306_DrawPixel(x0 - y, y0 - x, c);
    }
}

void ssd1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR c)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    ssd1306_DrawPixel(x0, y0 + r, c);
    ssd1306_DrawPixel(x0, y0 - r, c);
    ssd1306_DrawPixel(x0 + r, y0, c);
    ssd1306_DrawPixel(x0 - r, y0, c);
    ssd1306_DrawLine(x0 - r, y0, x0 + r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ssd1306_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, c);
        ssd1306_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, c);

        ssd1306_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, c);
        ssd1306_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, c);
    }
}

void ssd1306_ON(void)
{
	ssd1306_WriteCommand(0x8D);  
	ssd1306_WriteCommand(0x14);  
	ssd1306_WriteCommand(0xAF);  
}

void ssd1306_OFF(void)
{
	ssd1306_WriteCommand(0x8D);  
	ssd1306_WriteCommand(0x10);
	ssd1306_WriteCommand(0xAE);  
}


