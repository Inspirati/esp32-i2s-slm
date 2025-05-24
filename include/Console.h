#pragma once

void console_init(void);
void console_loop(void);

#define SERIAL_SPEED 115200

extern bool verbose;
extern bool enabled;
extern bool oneshot;

//#define USE_SERIAL_CONSOLE
#ifdef USE_SERIAL_CONSOLE

//#define USE_CPP  // if your compiler has trouble with, or you don't want to incur stdlib overhead, comment this out
#ifdef USE_CPP
#include <functional>
#include <vector>
#include <string>
#else
#endif // USE_CPP

#ifdef USE_CPP
struct Cmd {
	int index;
	std::function<void(const char*)> fptr;  // Use std::function to store lambdas or functions
	const char* cmdstr;
};
#else
class Console;
typedef struct console_tagCmds {
	int index;
	void (Console::*fptr)(const char*);  // Pointer to a member function
	const char* cmdstr;
} console_cmds_t;
#endif

class Console : public Print
{
public:
#ifdef USE_CPP
	Console(bool echo = false) : Print(), echo_on(echo) {
		cmdBuffer.clear();
		cmdslist = {
			{ 0, [&](const char* arg) { cmd_help(arg); }, "help" },
			{ 0, [&](const char* arg) { cmd_info(arg); }, "info" },
			{ 0, [&](const char* arg) { cmd_echo(arg); }, "echo" },
			{ 0, [&](const char* arg) { cmd_app(arg); }, "app" },
			{ 0, [&](const char* arg) { cmd_ver(arg); }, "ver" },
			{ 0, [&](const char* arg) { cmd_on(arg); }, "on" },
			{ 0, [&](const char* arg) { cmd_off(arg); }, "off" },
			{ 0, [&](const char* arg) { cmd_reset(arg); }, "reset" },
			{ 0, [&](const char* arg) { cmd_resume(arg); }, "resume" },
			{ 0, [&](const char* arg) { cmd_suspend(arg); }, "suspend" },
			{ 0, [&](const char* arg) { cmd_verbose(arg); }, "verbose" }
		};
	}
#else
	Console(bool echo = false) : cmdlen(0), echo_on(echo) {}
#endif // USE_CPP
	void loop(void);
	size_t write(const uint8_t *buffer, size_t size) override {
		Serial.write(buffer, size);
		return size;
	}
	size_t write(const char *buffer) {
		return write((uint8_t*)buffer, strlen(buffer));
	}
	size_t write(uint8_t val) {
		Serial.write(val);
		return 1;
	}

protected:
#ifdef USE_CPP
	//void command(const std::string& cmdstr);
	void command(std::string cmdstr);
	void inbyte(char ch);
#else
	void command(char* cmdstr, unsigned int cmdlen);
	void inbyte(char ch);
	char cmdstr_buf[32];
	unsigned int cmdlen;
#endif // USE_CPP
	void cmd_help(const char* arg);
	void cmd_info(const char* arg);
	void cmd_echo(const char* arg);
	void cmd_ver(const char* arg);
	void cmd_app(const char* arg);
	void cmd_on(const char* arg);
	void cmd_off(const char* arg);
	void cmd_reset(const char* arg);
	void cmd_resume(const char* arg);
	void cmd_suspend(const char* arg);
	void cmd_verbose(const char* arg);
#ifdef USE_CPP
	std::vector<Cmd> cmdslist;  // Use a vector for flexibility
	std::string cmdBuffer = std::string(32, '\0');  // Preallocated buffer
#else
	console_cmds_t cmdslist[11] = {
		{ 0, &Console::cmd_help,   "help" },
		{ 0, &Console::cmd_info,   "info" },
		{ 0, &Console::cmd_ver,    "ver" },
		{ 0, &Console::cmd_app,    "app" },
		{ 0, &Console::cmd_on,     "on" },
		{ 0, &Console::cmd_off,    "off" },
		{ 0, &Console::cmd_echo,   "echo" },
		{ 0, &Console::cmd_reset,  "reset" },
		{ 0, &Console::cmd_resume, "resume" },
		{ 0, &Console::cmd_suspend,"suspend" },
		{ 0, &Console::cmd_verbose,"verbose" }
	};
#endif // USE_CPP
	bool echo_on;
};

#endif // USE_SERIAL_CONSOLE

