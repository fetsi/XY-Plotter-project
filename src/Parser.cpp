/*
 * Parser.cpp
 *
 *  Created on: 20.10.2017
 *      Author: micromikko
 */

#include "Parser.h"
#include <cstring>
#include <stdlib.h>
//#include "Motor.h"		// ???
#include <cstdio>			// DEBUG
#include "ITM_write.h"		// DEBUG

Parser::Parser() {

}

Parser::~Parser() {
}

void Parser::m10Parse(PlotterData &plotdat, char *c_comstr) {
	/*
	 * -.,-.,-., printtaa vaan initialization juttuja.
	 */
}

void Parser::laserParse(PlotterData &plotdat, char *c_comstr) {
	plotdat.targetLaser = atoi(c_comstr);
}

void Parser::penParse(PlotterData &plotdat, char *c_comstr) {
	plotdat.targetPen = atoi(c_comstr);
}

void Parser::mcodeParse(PlotterData &plotdat, char *c_comstr) {
	char *pEnd;
	uint8_t comNum = strtod(c_comstr, &pEnd);
	c_comstr = pEnd;

	switch(comNum) {
	case 1:
		plotdat.gormNum = 1;
		penParse(plotdat, c_comstr + 1);
		break;
	case 4:
		plotdat.gormNum = 4;
		laserParse(plotdat, c_comstr + 1);
		break;
	case 10:
		plotdat.gormNum = 10;
		m10Parse(plotdat, c_comstr + 1);
	}
}

void Parser::xyParse(PlotterData &plotdat, char *c_comstr) {

	char *pEnd;
	plotdat.targetX = strtod(c_comstr, &pEnd);
	c_comstr = pEnd + 2;

	plotdat.targetY = strtod(c_comstr, &pEnd);
	c_comstr = pEnd + 2;
	
	plotdat.auxDelay = strtol(c_comstr, &pEnd, 10);
}

void Parser::gcodeParse(PlotterData &plotdat, char *c_comstr) {
	char *pEnd;
	uint8_t comNum = strtod(c_comstr, &pEnd);
	c_comstr = pEnd;

	switch(comNum) {
	case 1:
		plotdat.gormNum = 1;
		xyParse(plotdat, c_comstr + 2);
		break;
	case 28:
		plotdat.gormNum = 28;
	}
}

void Parser::generalParse(PlotterData &plotdat, std::string commandString) {
	char c_comstr[commandString.length() + 1];
	strcpy(c_comstr, commandString.c_str());

	switch(c_comstr[0]) {
	case 'G':
		plotdat.gorm = 'G';
		gcodeParse(plotdat, c_comstr + 1);
		break;
	case 'M':
		plotdat.gorm = 'M';
		mcodeParse(plotdat, c_comstr + 1);
	}
}

void Parser::debug(PlotterData &plotdat, std::string stringula, bool showAll) {

		char commandBuffer[200];
		memset(commandBuffer, 0, sizeof(commandBuffer));
		strcpy(commandBuffer, stringula.c_str());

		generalParse(plotdat, commandBuffer);
	
		if(showAll) {
			const char *format = "gorm: %c\r\ngormNum: %d\r\ntargetX: %.2f\r\ntargetY: %.2f\r\nauxDelay: %d\r\ntargetPen: %d\r\ntargetLaser: %d\r\n";
			memset(commandBuffer, 0, sizeof(commandBuffer));
			snprintf(commandBuffer, sizeof(commandBuffer), format, plotdat.gorm, plotdat.gormNum,
					plotdat.targetX, plotdat.targetY, plotdat.auxDelay, plotdat.targetPen, plotdat.targetLaser);
			ITM_write(commandBuffer);
		} else {
			if(plotdat.gorm == 'G') {
				if(plotdat.gormNum == 1) {
					const char *format = "%c%d X%.2f Y%.2f A%d\r\n";
					memset(commandBuffer, 0, sizeof(commandBuffer));
					snprintf(commandBuffer, sizeof(commandBuffer), format, plotdat.gorm, plotdat.gormNum,
							plotdat.targetX, plotdat.targetY, plotdat.auxDelay);
					ITM_write(commandBuffer);
				} else if(plotdat.gormNum == 28) {
					const char *format = "%c%d\r\n";
					memset(commandBuffer, 0, sizeof(commandBuffer));
					snprintf(commandBuffer, sizeof(commandBuffer), format, plotdat.gorm, plotdat.gormNum);
					ITM_write(commandBuffer);
				}
			} else if(plotdat.gorm == 'M') {
				if(plotdat.gormNum == 1) {
					const char *format = "%c%d %d\r\n";
					memset(commandBuffer, 0, sizeof(commandBuffer));
					snprintf(commandBuffer, sizeof(commandBuffer), format, plotdat.gorm, plotdat.gormNum, plotdat.targetPen);
					ITM_write(commandBuffer);
				} else if(plotdat.gormNum == 4) {
					const char *format = "%c%d %d\r\n";
					memset(commandBuffer, 0, sizeof(commandBuffer));
					snprintf(commandBuffer, sizeof(commandBuffer), format, plotdat.gorm, plotdat.gormNum, plotdat.targetPen);
					ITM_write(commandBuffer);
				} else if(plotdat.gormNum == 10) {
					const char *format = "%c%d\r\n";
					memset(commandBuffer, 0, sizeof(commandBuffer));
					snprintf(commandBuffer, sizeof(commandBuffer), format, plotdat.gorm, plotdat.gormNum);
					ITM_write(commandBuffer);
				}
			}
		}
		ITM_write("+++\r\n\r\n");
}
