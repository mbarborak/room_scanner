// IMPORTANT: ELEGOO_TFTLCD LIBRARY MUST BE SPECIFICALLY
// CONFIGURED FOR EITHER THE TFT SHIELD OR THE BREAKOUT BOARD.
// SEE RELEVANT COMMENTS IN Elegoo_TFTLCD.h FOR SETUP.
//Technical support:goodtft@163.com

#include <Elegoo_GFX.h>    // Core graphics library
#include <Elegoo_TFTLCD.h> // Hardware-specific library

#include <Ultrasonic.h>
#include <Servo.h>         // include Servo library

// Servo stuff.
Servo servo;
int servoPos = 20;
int servoPosMax = 160;
int servoPosMin = 20;
int servoDelta = 1;

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

Elegoo_TFTLCD tft(A3, A2, A1, A0, A4);
Ultrasonic us(39, 37, 40000UL); // (Trig PIN,Echo PIN, timeout)

unsigned long lastMoveTime;

char distStr[10];

void clearDistDisp() {
	tft.setTextColor(BLACK);
	tft.setCursor(0, 0);
	tft.print(distStr);
}

void printDistDisp(int cms) {
	clearDistDisp();
	float insExact = 0.393701 * cms;
	int ft = insExact / 12;
	float ins = fmod(insExact, 12.0);
	float insRemainder = ins - (int) ins;
	int wholeIns = (int) ins;
	int remainderDigit = 0;
	if (insRemainder > .25 && insRemainder < .75) remainderDigit = 5;
	else if (insRemainder >= .75) wholeIns++;

	sprintf(distStr, "%d'%d.%d\"", ft, wholeIns, remainderDigit);
	tft.setTextColor(GREEN);
	tft.setCursor(0, 0);
	tft.print(distStr);
}

unsigned int sampleCt;
unsigned int samples[1000];

void resetSamples() {
	sampleCt = 0;
}

unsigned int sample() {
	unsigned int cms = us.distanceRead();
	samples[sampleCt++] = cms;
	return cms;
}

int compareInt( const void* a, const void* b ) {
    if( *(unsigned int*)a == *(unsigned int*)b ) return 0;
    return *(unsigned int*)a < *(unsigned int*)b ? -1 : 1;
}

unsigned int sampleDist() {
	qsort(samples, sampleCt, sizeof(unsigned int), compareInt);
	unsigned int result = samples[sampleCt / 2];
	Serial.print(result);
	Serial.print(" ");
	Serial.print(sampleCt / 2);
	Serial.println("");
	resetSamples();
	return result;
}

const float degToRad = 3.141592654f / 180;
const float ang0 = servoPosMin * degToRad;
const float angDelta = abs(servoDelta) * degToRad;

void drawRoomEdges(unsigned int *dists, unsigned int maxDistsIndex, unsigned int maxDist, unsigned int index0, unsigned int index1, unsigned int color) {
	int lastX;
	int lastY;
	unsigned int loNdx = min(index0, index1);
	unsigned int hiNdx = max(index0, index1);
	float ang = ang0 + angDelta * loNdx;
	for (unsigned int i = loNdx; i >= 0 && i <= maxDistsIndex && i <= hiNdx; i++) {
		float r = (dists[i] * 1.0 / maxDist) * 119.0;
		int x = r * cos(ang) + 120;
		int y = r * sin(ang) + 140;
		if (i - loNdx > 0) {
			tft.drawLine(lastX, lastY, x, y, color);
		}
		ang += angDelta;
		lastX = x;
		lastY = y;
	}
}

void drawRoom(unsigned int *dists, unsigned int maxDistsIndex, unsigned int maxDist, unsigned int color) {
	drawRoomEdges(dists, maxDistsIndex, maxDist, 0, maxDistsIndex, color);
}

void setup(void) {
	Serial.begin(115200);

	// Ultrasonic
	pinMode(41, OUTPUT); // VCC pin
	pinMode(35, OUTPUT); // GND ping
	digitalWrite(41, HIGH); // VCC +5V mode
	digitalWrite(35, LOW);  // GND mode

	// Servo
	servo.attach(50);
	servoPos = servoPosMin;
	servo.write(servoPos);
	delay(1000);

	// Screen
	tft.reset();
	uint16_t identifier = tft.readID();
	if (identifier == 0x0101) {
		identifier = 0x9341;
		Serial.println(F("Found 0x9341 LCD driver"));
	}
	tft.begin(identifier);
	tft.fillScreen(BLACK);
	tft.setTextSize(5);

	lastMoveTime = millis();
	resetSamples();
}


unsigned int maxDist = 0;
unsigned int dists[360];
unsigned int maxDistsIndex = 0;
unsigned int cycles = 0;

void loop(void) {
	if (millis() - lastMoveTime > 1000) {
		unsigned int dist = sampleDist();

		Serial.print(dist);
		Serial.println("");

		if (dist > 0) {
			unsigned int index = (servoPos - servoPosMin) / abs(servoDelta);
			maxDistsIndex = max(maxDistsIndex, index);

			unsigned int lastMaxDist = maxDist;
			maxDist = 0;
			for (unsigned int i = 0; i <= maxDistsIndex; i++) {
				maxDist = max(maxDist, i == index ? dist : dists[i]);
			}

			if (lastMaxDist != maxDist) {
				Serial.println("erase");
				drawRoom(dists, maxDistsIndex, lastMaxDist, BLACK);
			} else {
				drawRoomEdges(dists, maxDistsIndex, lastMaxDist, index - 1, index + 1, BLACK);
			}

			dists[index] = cycles == 0 ? dist : (dists[index] * 1.0 * cycles + dist) / (cycles + 1.0);

			if (lastMaxDist != maxDist) {
				drawRoom(dists, maxDistsIndex, maxDist, RED);
			} else {
				drawRoomEdges(dists, maxDistsIndex, maxDist, index - 1, index + 1, RED);
			}

			servoPos += servoDelta;
			if (servoPos < servoPosMin || servoPos > servoPosMax) {
				servoDelta = -servoDelta;
				servoPos += 2 * servoDelta;
				cycles++;
			}
			servo.write(servoPos);
			delay(100);
		}

		lastMoveTime = millis();
	}

	unsigned int cms = sample();
	if (cms >= 0) printDistDisp(cms);
	delay(10);
}

