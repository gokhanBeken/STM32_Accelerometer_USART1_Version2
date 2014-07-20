#include "STM32F4xx.h"

#include <math.h> //dereceye çevirmek için
#define PI_SAYISI 3.14159
#define ORNEKLEME_SAYISI 10
//dasdasd

unsigned char x, y, z; //ham degerler, 0 ila 255 arasi veri aliyor
unsigned char xKayit[ORNEKLEME_SAYISI], yKayit[ORNEKLEME_SAYISI],
		zKayit[ORNEKLEME_SAYISI];
float x_derece, y_derece, z_derece; //bunlarda donusturecegimiz degerler

unsigned int x_dene, y_dene, z_dene; //çevirme iþleminde kullanýyorum

char s_str[12] = "";

//bayraklar
char x_durum = 0, y_durum = 0; //koordinat bayraklarimiz
volatile int durum = 0; //timerdeki xy durumunu global yapmak icin

void UsartInit(void);
void SendChar(char Tx);
void SendTxt(char *Adr);

void donustur_str(unsigned long sayi);

unsigned char KalmanFiltreOrtalamasiX(void);
unsigned char KalmanFiltreOrtalamasiY(void);

void DelayMs(int miliseconds);

char PinTest(unsigned char port, char bit);

/*****************************************************************************************************
 CPU PLL ile 168Mhz de kosturulur
 AHB frekansy 168 Mhz
 APB1 frekansy 42 Mhz
 APB2 frekansy 84 Mhz
 *****************************************************************************************************/
void SystemInit2() {
	volatile unsigned int i;
	(*((int*) 0xE000ED88)) |= 0x0F00000;
	for (i = 0; i < 0x00100000; i++)
		; // OSC oturtma ve kurtarma rutini
	RCC->CFGR |= 0x00009400; // AHB ve APB hizlarini max degerlere set edelim
	RCC->CR |= 0x00010000; // HSE Xtal osc calismaya baslasin
	while (!(RCC->CR & 0x00020000))
		; // Xtal osc stabil hale gelsin
	RCC->PLLCFGR = 0x07402A04; // PLL katsayilarini M=4, N=168, P=2 ve Q=7 yapalim
	RCC->CR |= 0x01000000; // PLL calismaya baslasin  (Rehber Sayfa 95)
	while (!(RCC->CR & 0x02000000))
		; // Pll hazir oluncaya kadar bekle
	FLASH->ACR = 0x00000605; // Flash ROM icin 5 Wait state secelim ve ART yi aktif edelim (Rehber Sayfa 55)
	RCC->CFGR |= 0x00000002; // Sistem Clk u PLL uzerinden besleyelim
	while ((RCC->CFGR & 0x0000000F) != 0x0000000A)
		; // Besleninceye kadar bekle
	RCC->AHB1ENR |= 0x0000001F; // GPIO A,B,C,D,E clock'u aktif edelim
	GPIOD->MODER = 0x55550000; // GPIOD nin 15, 14, 13, 12, 11, 10, 9, 8 pinleri cikis tanimlandi (LEDler icin)
	GPIOD->OSPEEDR = 0xFFFFFFFF; // GPIOD nin tum cikislari en yuksek hizda kullanacagiz
//  GPIOA da A7, A6, A5 pinleri, LIS302DL cipiyle haberlesmek icin SPI moduna alinacak

	GPIOA->AFR[0] = 0x55500000; // SPI1 secelim (Rehber Sayfa 141), Hard Sayfa 49
	GPIOA->MODER |= 0x0000A800; // A7,A6,A5 Alternatif fonksiyon
	RCC->APB2ENR |= 0x00001000; // SPI1 clk enable   //   GPIOE3 pinini output tanimlayalim (LIS302DL SPI/I2C secimi)
	GPIOE->MODER = 0x00000040; // GPIOE nin 3 nolu pini cikis tanimlandi
	GPIOE->OSPEEDR = 0xFFFFFFFF; // GPIOE nin tum cikislari en yuksek hizda kullanacagiz
	GPIOE->BSRRL = 0x0008; // GPIOE3=1; LIS302DL CS=1
	SPI1->CR1 = 0x00000B7F; // SPI1 16 bit, master , fulldublex vs
	SPI1->CR2 = 0X0000;

	RCC->APB1ENR |= 0x00000020; // Timer7 CLK'u aktif edelim (84 Mhz)
	TIM7->CR1 = 0x0080; // Otomatik Reload
	TIM7->PSC = 839; // Prescaler degerimiz 839, Count frekansimiz = fCK_PSC / (Yuklenen Deger + 1) 84E6 / (840) = 100 KHz
	TIM7->ARR = 1; // Counter, Decimal 1 olunca basa donsun. Her 20 mikrosaniye de bir timer int olusacak.
	TIM7->DIER = 0x0001; // Update Int enable
	NVIC->ISER[1] = 0X00800000; // NVIC de Timer 7 interrupta izin verelim
	TIM7->CR1 |= 0x0001; // Counter Enable

	GPIOA->MODER &= ~0x00000003; //a0 butonunu giris yapalim(00=input)

}
void TIM7_IRQHandler() {

	TIM7->SR = 0; // Timer Int Flagini silelim

}
signed char SPI_CMD(short DAT) {
	signed char RxDat;
	GPIOE->BSRRH = 0x0008; // LIS302DL CS=0
	RxDat = SPI1->SR; // AMAC DELAY (kalmasinda fayda var)
	SPI1->DR = DAT; // Komutu yukle
	while (!(SPI1->SR & 0x01))
		; // RX BUF bos ise beklemede kal
	while (SPI1->SR & 0x80)
		; // BSY durumu varsa kalkmasini bekleyelim
	RxDat = SPI1->DR; // Cipten gelen veriyi oku
	while (SPI1->SR != 0x02)
		; // CS=1 yapmadan once cipin orjinal duruma donmeyi bekleyelim
	GPIOE->BSRRL = 0x0008; // LIS302DL CS=1
	return (RxDat);
}

void delay(unsigned int time) {
	while (time--)
		;
}
void Write(char Adr, unsigned char Data) {
	GPIOE->BSRRH = 0x0008; // LIS302DL CS=0
	SPI1->DR = (unsigned short) (((Adr & 0x3F) << 8) | Data);
	while (!(SPI1->SR & 2))
		;
	delay(10000);
	GPIOE->BSRRL = 0x0008; // LIS302DL CS=1
}

/*
 signed char Write(char Adr, unsigned char Data) {
 return (SPI_CMD(((Adr & 0x3F) << 8) | Data));
 }
 */
signed char Read(char Adr) {
	return (SPI_CMD(((Adr & 0x3F) | 0x80) << 8));
}

int main() {
	SystemInit2();
	signed char who;
	char i;

	double denex;

	UsartInit();

	SendTxt("start \r\n");

	if (Read(0x0F) == 0x3B) // Who are you ?
			{
		Write(0x20, 0x47); // Data Rate=100Hz, Full Scale=2g, Activate, x,y,z enable
		while (1) {
			who = Read(0x27); // Statusu ogrenelim. Kim hazir kim deðil?
			if (who & 1)
				x = Read(0x29);
			if (who & 2)
				y = Read(0x2B);
			if (who & 4)
				z = Read(0x2D);



			//her gelen veriyi en sona aktarip, eski verileri bir bir kaydirma algoritmasi(kalman filtresi):
			for (i = 0; i < ORNEKLEME_SAYISI; i++) {
				xKayit[i] = xKayit[i + 1];
			}
			xKayit[ORNEKLEME_SAYISI-1] = x; //en son taze veri(-1 yapmamýzýn sebebi, örn: 10 elemanlýk bir dizinin en son elamaný 9 dur, bu yüzden)


			for (i = 0; i < ORNEKLEME_SAYISI; i++) {
				yKayit[i] = yKayit[i + 1];
			}
			yKayit[ORNEKLEME_SAYISI-1] = y; //en son taze veri
			//if(y > yKayit[ORNEKLEME_SAYISI-1] && ((y - yKayit[ORNEKLEME_SAYISI-1]) < 100 || (y - yKayit[ORNEKLEME_SAYISI-1]) > 200)){yKayit[ORNEKLEME_SAYISI-1] = y;}
			//if(y < yKayit[ORNEKLEME_SAYISI-1] && ((yKayit[ORNEKLEME_SAYISI-1]-y) < 100 || yKayit[ORNEKLEME_SAYISI-1]-y) > 200){yKayit[ORNEKLEME_SAYISI-1] = y;}




			x = KalmanFiltreOrtalamasiX();
			y = KalmanFiltreOrtalamasiY();

			//denex = sqrt((double) y);

			//x_derece=(atan(x / sqrt((1.0 * y * y) + (z * z))) * 180) / PI_SAYISI;
			//x_derece = round(x_derece);

			//aldýðýmýz ham verileri, gerçek açýya çeviriyoruz(360/255=1.4 küsür olduðu için, amacýmýz ham veriyi bu sayýyla çarpmak)
			x_derece = x;
			x_derece *= (float) 1.4117;
			x_dene = (int) x_derece; //x_dene degiskenine yaziyoruz, cunku x degeri char olmak zorunda int olursa ham veriyi yanlis aliyor. gerçek açý deðerini ise char olduðu için alamýyor bu yüzden int tipinde olan x_dene deðiþkenini kullanýyorum

			y_derece = y;
			//y_derece *= (float) 1.4;
			y_derece *= (float)1.4117;//1,411764705882353;
			y_dene = (int) y_derece;




			SendTxt("x=");
			donustur_str(x_dene);
			SendTxt(s_str);

			SendTxt(",");

			SendTxt("y=");
			donustur_str(y_dene);
			SendTxt(s_str);

			SendTxt(",");

			SendTxt("z=0");
			SendTxt("\r\n");


			DelayMs(50);
			//for (i = 0; i < 0x1000000; i++) { }//bekle
		}
	}

	while (1)
		;

}

void UsartInit(void) {
//   USART3 MODULUNU AKTIF HALE GETIRELIM

	RCC->APB1ENR |= 0x00040000; // USART3 Clk Enable (Rehber Sayfa 113)
	RCC->APB1RSTR |= 0x00040000; // USART3 Resetlendi
	GPIOB->AFR[1] = 0x07777700; // PB10..PB14 pinleri USART3 ile alakalandirildi (Hard Sayfa 49)
	GPIOB->MODER |= 0x2AA00000; // GPIOB 10..14 icin alternatif fonksiyon tanimi (Rehber Sayfa 148)

//   USART3 MODULUNU AYARLAYALIM      // 1 Start, 8 Data, 1 Stop, No parity (Default degerler)

	RCC->APB1RSTR &= ~0x00040000; // USART3 Reseti kaldiralim
	USART3->SR &= ~0X03FF; // Status registeri silelim
	USART3->BRR = 0X1112; // 9600 Baud

	USART3->CR1 |= 0x0000200C; // USART3 enable
}

void SendChar(char Tx) {
	while (!(USART3->SR & 0x80))
		; // TX Buffer dolu ise bekle (Rehber Sayfa 646)
	USART3->DR = Tx;
}

void SendTxt(char *Adr) {
	while (*Adr) {
		SendChar(*Adr);
		Adr++;
	}
}

void DelayMs(int miliseconds) {
	int i = 0, j = 0;
	for (i = 0; i < miliseconds; i++) {
		for (j = 0; j < 8388; j++) {

		}
	}
}

void donustur_str(unsigned long sayi) {
	int sayi_2 = 0;
	int boy = 0, don = 0;

	char s_str_yedek[12] = "";
	s_str[0] = 0;
	s_str[0] = 0;
	s_str[1] = 0;
	s_str[2] = 0;
	s_str[3] = 0;

	for (don = 0; don < 50; don++) {
		sayi_2 = sayi % 10;
		sayi = sayi - sayi_2;
		sayi = sayi / 10;
		s_str_yedek[don] = sayi_2 + 48;
		s_str_yedek[don + 1] = 0;
		boy = don;
		if (sayi < 1) {
			break;
		}
	}

	for (don = 0; don < (boy + 1); don++) {
		s_str[don] = s_str_yedek[boy - don];
		s_str[don + 1] = 0;
	}
}

unsigned char KalmanFiltreOrtalamasiX(void) {
	int i = 0;
	int toplam = 0;
	float ortalama = 0;
	for (i = 0; i < ORNEKLEME_SAYISI; i++) {
		if(xKayit[i]>127){toplam -= (255-xKayit[i]); }
		else {toplam += xKayit[i];}
	}



	ortalama = (float) toplam / ORNEKLEME_SAYISI;

	if(ortalama<0)ortalama+=255;

	//if ((ortalama - (unsigned char) ortalama) >= 0.5) {
		//ortalama += 1;
	//}


	return ((unsigned char) ortalama);
}

unsigned char KalmanFiltreOrtalamasiY(void) {
	int i = 0;
	int toplam = 0;
	float ortalama = 0;
	for (i = 0; i < ORNEKLEME_SAYISI; i++) {
		if(yKayit[i]>127){toplam -= (255-yKayit[i]); }
		else {toplam += yKayit[i];}
	}

	ortalama = (float) toplam / ORNEKLEME_SAYISI;

	if(ortalama<0)ortalama+=255;

	//if ((ortalama - (unsigned char) ortalama) >= 0.5) {
		//ortalama += 1;
	//}

	return ((unsigned char) ortalama);
}

/*
unsigned char KalmanFiltreOrtalamasiX(void) {
	int i = 0;
	unsigned int toplam = 0;
	float ortalama = 0;
	for (i = 0; i < ORNEKLEME_SAYISI; i++) {
		toplam += xKayit[i];
	}
	ortalama = (float) toplam / ORNEKLEME_SAYISI;
	if ((ortalama - (unsigned char) ortalama) >= 0.5) {
		return (ortalama + 1);
	}
	return ((unsigned char) ortalama);
}


unsigned char KalmanFiltreOrtalamasiY(void) {
	int i = 0;
	unsigned int toplam = 0;
	float ortalama = 0;
	for (i = 0; i < ORNEKLEME_SAYISI; i++) {
		toplam += yKayit[i];
	}
	ortalama = (float) toplam / ORNEKLEME_SAYISI;
	if ((ortalama - (unsigned char) ortalama) >= 0.5) {
		return (ortalama + 1);
	}
	return ((unsigned char) ortalama);
}
*/
char PinTest(unsigned char port, char bit) {
	char sonuc = 0;

	if (port == 'a' || port == 'A') {
		if (!(((GPIOA->IDR) & (1 << bit)) - (1 << bit))) { //butona basildi mi
			sonuc = 1;
		} else {
			sonuc = 0;
		}
	}

	return (sonuc);

	/*
	 pinset, pinclr ve pininp fonksiyonlarinin her birisi hata dondurme yetenegine sahiptir.

	 Hata degeri 2 olarak geri donulduyse olmayan bir GPIO dan yada olmayan bir pin numarasindan bahsediyoruz demektir.

	 pinset('J',0) yada pinclr('A',20) 2 hatasi ile geri donuse neden olur.
	 */
}
