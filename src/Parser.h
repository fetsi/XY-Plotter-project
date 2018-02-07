/*
 * Parser.h
 *
 *  Created on: 20.10.2017
 *      Author: micromikko
 */

#ifndef PARSER_H_
#define PARSER_H_

#include <string>
#include "PlotterData.h"


class Parser {
public:
	Parser();
	virtual ~Parser();

	void generalParse(PlotterData &plotdat, const std::string commandString);
	void gcodeParse(PlotterData &plotdat, char *c_comstr);
	void mcodeParse(PlotterData &plotdat, char *c_comstr);
	void xyParse(PlotterData &plotdat, char *c_comstr);

	void penParse(PlotterData &plotdat, char *c_comstr);
	void laserParse(PlotterData &plotdat, char *c_comstr);
	void m10Parse(PlotterData &plotdat, char *c_comstr);

	void debug(PlotterData &plotdat, std::string stringula, bool showAll=false);

private:

};

#endif /* PARSER_H_ */
