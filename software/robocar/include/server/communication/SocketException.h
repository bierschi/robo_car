//
// Created by christian on 03.09.18.
//

#ifndef ROBOCAR_SOCKETEXCEPTION_H
#define ROBOCAR_SOCKETEXCEPTION_H

/**
 * /CLASS SocketException
 *
 * handles Socket Exception
 *
 */
class SocketException {

private:
    std::string m_s;

public:

    SocketException( std::string s ) : m_s(s) {}
    ~SocketException(){}

    std::string description() {return m_s;}

};

#endif //ROBOCAR_SOCKETEXCEPTION_H
