//
// Created by christian on 03.09.18.
//

#include <cstring>
#include <fcntl.h>
#include "server/communication/Socket.h"

/**
 * BaseClass Constructor for a socket instance
 */
Socket::Socket() : m_sock(-1){

    memset(&m_addr, 0, sizeof(m_addr));

}

/**
 * BaseClass Destructor to cleanly close socket connections
 */
Socket::~Socket() {

    if ( isValid() ) {

        ::close(m_sock);

    }
}

/**
 * creates a AF_INET, SOCK_STREAM socket
 *
 * @return true, if server socket could be created, else false
 */
bool Socket::create() {

    m_sock = socket(AF_INET, SOCK_STREAM, 0);

    if ( !isValid() )
        return false;

    int on = 1;
    if ( setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&on, sizeof(on)) == -1 )
        return false;

    return true;
}

/**
 * bind socket to given port
 *
 * @param port: const int
 * @return true, if port could be bind to socket, else false
 */
bool Socket::bind(const unsigned int port) {

    if ( !isValid() )
        return false;

    m_addr.sin_family = AF_INET;
    m_addr.sin_addr.s_addr = INADDR_ANY;
    m_addr.sin_port = htons( port );

    int bind_return = ::bind(m_sock, (struct sockaddr*) &m_addr, sizeof(m_addr));
    if (bind_return == -1)
        return false;

    return true;
}

/**
 * listen to created socket
 *
 * @return true, if socket could be listened, else false
 */
bool Socket::listen() const {

    if ( !isValid() )
        return false;

    int listen_return = ::listen(m_sock, MAXCONNECTIONS);
    if (listen_return == -1)
        return false;

    return true;
}

/**
 * accepts client connections
 *
 * @param new_socket: Socket
 * @return true, if socket was accepted, else false
 */
bool Socket::accept(Socket & new_socket) const {

    int addr_length = sizeof(m_addr);

    new_socket.m_sock = ::accept(m_sock, (sockaddr*)&m_addr, (socklen_t *) &addr_length);
    std::clog << "new client is connecting with socket_fd: " << new_socket.m_sock << std::endl;

    if (new_socket.m_sock <= 0)
        return false;
    else
        return true;
}

/**
 * send string to socket
 *
 * @param s: std::string
 * @return true, if sending was successfully, else false
 */
bool Socket::send(const std::string s) const {

    int status = ::send(m_sock, s.c_str(), s.size(), MSG_NOSIGNAL);
    std::clog << "send str-msg ..." << std::endl;

    if (status == -1)
        return false;
    else
        return true;
}

/**
 * send command to socket
 *
 * @param cmd: Commands& reference
 * @return true, if sending was successfully, else false
 */
bool Socket::send(Commands& cmd) const {

    int status = ::send(m_sock, &cmd, sizeof(cmd), MSG_NOSIGNAL);
    std::clog << "send cmd-msg ..." << std::endl;

    if (status == -1)
        return false;
    else
        return true;
}

bool Socket::send(std::vector<int>& v) {

    //const std::int64_t sz = v.size();
    long sz = v.size();
    std::cout << "vectorsize: " <<v.size() << std::endl;
    std::cout << "sizeof(): " << sizeof(sz) << std::endl;
    int status = ::send(m_sock, &sz, sizeof(sz), MSG_NOSIGNAL);
    status = ::send(m_sock, &(*v.begin()), sz * sizeof(int), MSG_NOSIGNAL);
    std::clog << "send vector .." << std::endl;

    if (status == -1 )
        return false;
    else
        return true;

}

/**
 * receive string from socket
 *
 * @param s: std::string
 * @return true, if receiving was successfully, else false
 */
int Socket::recv(std::string& s) const {

    char buf [MAXRECV +1 ];

    s = "";

    memset(buf, 0, MAXRECV +1 );

    int status = ::recv(m_sock, buf, MAXRECV, 0);
    std::clog << "recv msg ..." << std::endl;

    if (status == -1) {

        std::cout << "status == -1 errno == " << errno << " in Socket::recv()\n";

    } else if( status == 0) {

        return 0;

    } else {

        s = buf;

        return status;

    }

}

/**
 * receive command from socket
 *
 * @param cmd: Commands& reference
 * @return true, if sending was successfully, else false
 */
int Socket::recv(Commands& cmd) const {


    int status = ::recv(m_sock, &cmd, MAXRECV, 0);
    std::clog << "send msg ..." << std::endl;

    if (status == -1) {

        std::cout << "status == -1 errno == " << errno << " in Socket::recv()\n";

    } else if( status == 0) {

        return 0;

    } else {

        return status;

    }

}

/**
 * connecting to a particular socket
 *
 * @param host: std::string
 * @param port: const int port
 * @return true, if connecting was successfully, else false
 */
bool Socket::connect(const std::string host, const int port) {

    if ( !isValid() )
        return false;

    m_addr.sin_family = AF_INET;
    m_addr.sin_port = htons(port);

    int status = inet_pton(AF_INET, host.c_str(), &m_addr.sin_addr);

    if (errno == EAFNOSUPPORT)
        return false;

    status = ::connect(m_sock, (sockaddr*)&m_addr, sizeof(m_addr));
    std::clog << "connecting to " << host << ", " << port << std::endl;

    if (status == 0)
        return true;
    else
        return false;
}

void Socket::setNonBlocking(const bool b) {

    int opts;

    opts = fcntl( m_sock, F_GETFL);

    if (opts < 0){
        return;
    }

    if (b)
        opts = (opts | O_NONBLOCK);
    else
        opts = (opts & ~O_NONBLOCK);

    fcntl(m_sock, F_SETFL, opts);
}