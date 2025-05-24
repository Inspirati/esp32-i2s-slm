#include <Arduino.h>
#include <string.h>
#include "Console.h"

bool verbose = false;  // enable echoing results to the serial monitor
bool enabled = true;   // not currently used by the application - see suspend/resume
bool oneshot = false;  // for diagnosing if the mic is returning data

#ifndef USE_SERIAL_CONSOLE
void console_init(void) {
	Serial.begin(115200);
	delay(1000); // Safety
}
void console_loop(void) {}
#else

#include "esp32-i2s-slm.h" // for slm resume and suspend

Console serialConsole(false);

void Console::cmd_app(const char* arg) {
	if (!*arg) {
		printf("'app help' for usage\n");
	} else {
		if (!strcmp("help", arg)) {
			Serial.print("oneshot (one shot dump of mic data)");
		}
		if (!strcmp("oneshot", arg)) oneshot = true;
		// insert other user app-lication specific stuff here ...
	}
}

void Console::cmd_ver(const char* arg) {
	Serial.printf("esp32-i2s-slm: " __DATE__ " @ " __TIME__ "\n");
	Serial.printf("ESP_IDF " IDF_VER "\n");
}

void Console::cmd_on(const char* arg) {
	printf("slm enabled.\r\n");
	enabled = true;
}

void Console::cmd_off(const char* arg) {
	printf("slm disabled.\r\n");
	enabled = false;
}

void Console::cmd_echo(const char* arg) {
	echo_on = !echo_on;
	if (echo_on) println("echo is on");
	else println("echo is off");
}

void Console::cmd_reset(const char* arg) {
#if defined(ESP8266)
	ESP.reset();
#elif defined(ESP32)
	ESP.restart();
#endif
}

void Console::cmd_resume(const char* arg) {
	printf("slm resumed.\r\n");
	slm_resume();
}

void Console::cmd_suspend(const char* arg) {
	printf("slm suspended.\r\n");
	slm_suspend();
}

void Console::cmd_verbose(const char* arg) {
	verbose = !verbose;
	if (verbose) println("verbose");
	else println("silent");
}

void Console::cmd_help(const char* arg) {
	printf("Usage:\n");
#ifdef USE_CPP
	for (const auto& c : cmdslist) {
		printf("\t%s\n", c.cmdstr);
	}
#else
	for (uint8_t i = 0; i < (sizeof(cmdslist)/sizeof(cmdslist[0])); i++) {
		printf("\t%s\r\n", cmdslist[i].cmdstr);
	}
#endif // USE_CPP
}

void Console::cmd_info(const char* arg) {
	printf("Cpu Freq: %u MHz\n", ESP.getCpuFreqMHz());
	printf("Free Heap: %u MB\n", ESP.getFreeHeap());
//	printf("Reset Reason: %s\n", ESP.getResetReason());
//	printf("Heap Fragmentation: %u\n", ESP.getHeapFragmentation());
//	printf("Flash Chip Real Size: %u\n", ESP.getFlashChipRealSize());
//	printf(": %s\n", );
}

#ifdef USE_CPP
void Console::command(std::string cmdstr) {
	std::string argstr;
	bool recognised = false;

	size_t spacePos = cmdstr.find(' ');
	if (spacePos != std::string::npos) {
		argstr = cmdstr.substr(spacePos + 1);  // Extract arguments
		cmdstr = cmdstr.substr(0, spacePos);   // Keep only command string
	}
	for (const auto& c : cmdslist) {
		if (cmdstr == c.cmdstr) {
			c.fptr(argstr.c_str());  // Directly call the function
			recognised = true;
			break;
		}
	}
	if (!recognised) {
		printf("Command '%s' not recognised\n", cmdstr.c_str());
	}
}
void Console::inbyte(char ch) {
	if (cmdBuffer.length() < (cmdBuffer.capacity() - 1)) {
		if (ch == '\r' || ch == '\n') {
			if (!cmdBuffer.empty()) {
				command(cmdBuffer);
			}
			cmdBuffer.clear();
		} else {
			cmdBuffer += ch;
		}
	} else {
		cmdBuffer.clear();
	}
}
#else
void Console::command(char* cmdstr, unsigned int cmdlen) {
	char* argstr = NULL;
	bool recognised = false;

	for (unsigned int i = 0; i < cmdlen; i++) {
		if (cmdstr[i] == ' ') {
			cmdstr[i] = '\0';
			if (i < (sizeof(cmdstr_buf)/sizeof(cmdstr[0]) - 1)) {
				argstr = cmdstr + i + 1;
			}
		}
	}
	for (unsigned int i = 0; i < (sizeof(cmdslist)/sizeof(cmdslist[0])); i++) {
		if (strcmp(cmdslist[i].cmdstr, cmdstr) == 0) {
			(this->*cmdslist[i].fptr)(argstr);
			recognised = true;
		}
	}
	if (!recognised) {
		printf("Command '%s' not recognised\n", cmdstr);
	}
}

void Console::inbyte(char ch) {
	static unsigned int cmdlen = 0;
	if (cmdlen < (sizeof(cmdstr_buf)/sizeof(cmdstr_buf[0]) - 1)) {
		cmdstr_buf[cmdlen] = ch;
		if ((ch == '\r') || (ch == '\n')) {
			cmdstr_buf[cmdlen] = '\0';
			if (strlen(cmdstr_buf) > 0) {
				command(cmdstr_buf, cmdlen);
			}
			cmdlen = 0;
		} else {
			cmdlen++;
		}
	} else {
		cmdlen = 0;
	}
}
#endif // USE_CPP

void console_init(void) {
	Serial.begin(SERIAL_SPEED);
	while (!Serial) {};
	Serial.println("type 'help' for usage (menu)");
}

char getch(void) {
	char buf[2];
	Serial.read(buf, 1);
	return buf[0];
}

void Console::loop(void) {
	if (Serial.available()) {
		char ch = getch();
		//Serial.printf("gotch: 0x%02x\n", ch);
		if (echo_on) write(ch);
		inbyte(ch);
	}
}

void console_loop(void) {
	serialConsole.loop();
}

#endif // USE_SERIAL_CONSOLE

