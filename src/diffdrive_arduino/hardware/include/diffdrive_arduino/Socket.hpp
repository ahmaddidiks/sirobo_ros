// Definition of the ServerSocket class

#ifndef Socket_class
#define Socket_class

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

const int MAXHOSTNAME = 200;
const int MAXCONNECTIONS = 5;
const int MAXRECV = 500;

class Socket
{
 public:
  Socket();
  virtual ~Socket();

  // Server initialization
  bool create();
  bool bind ( const int port );
  bool listen() const;
  bool accept ( Socket& ) const;

  // Client initialization
  bool connect ( const std::string host, const int port );

  // Data Transimission
  bool send ( const std::string ) const;
  int recv ( std::string& ) const;

  void set_non_blocking ( const bool );
  bool is_valid() const { return m_sock != -1; }

 private:
  int m_sock;
  sockaddr_in m_addr;
};

class ServerSocket : private Socket
{
 public:
  ServerSocket ( int port );
  ServerSocket (){};
  virtual ~ServerSocket();

  const ServerSocket& operator << ( const std::string& ) const;
  const ServerSocket& operator >> ( std::string& ) const;

  void accept ( ServerSocket& );

};

class ClientSocket : private Socket
{
 public:
  ClientSocket ( std::string host, int port );
  virtual ~ClientSocket(){};

  const ClientSocket& operator << ( const std::string& ) const;
  const ClientSocket& operator >> ( std::string& ) const;

};

class SocketException
{
 public:
  SocketException ( std::string s ) : m_s ( s ) {};
  ~SocketException (){};

  std::string description() { return m_s; }

 private:
  std::string m_s;
};


Socket::Socket() : m_sock(-1)
{
  memset(&m_addr,
         0,
         sizeof(m_addr));
}

Socket::~Socket()
{
  if (is_valid())
    ::close(m_sock);
}

bool Socket::create()
{
  m_sock = socket(AF_INET,
                  SOCK_STREAM,
                  0);

  if (!is_valid())
    return false;

  // TIME_WAIT - argh
  int on = 1;
  if (setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&on, sizeof(on)) == -1)
    return false;

  return true;
}

bool Socket::bind(const int port)
{
  if (!is_valid())
    return false;

  m_addr.sin_family = AF_INET;
  m_addr.sin_addr.s_addr = INADDR_ANY;
  m_addr.sin_port = htons(port);

  int bind_return = ::bind(m_sock,
                           (struct sockaddr *)&m_addr,
                           sizeof(m_addr));

  if (bind_return == -1)
    return false;

  return true;
}

bool Socket::listen() const
{
  if (!is_valid())
    return false;

  int listen_return = ::listen(m_sock, MAXCONNECTIONS);

  if (listen_return == -1)
    return false;

  return true;
}

bool Socket::accept(Socket &new_socket) const
{
  int addr_length = sizeof(m_addr);
  new_socket.m_sock = ::accept(m_sock, (sockaddr *)&m_addr, (socklen_t *)&addr_length);

  if (new_socket.m_sock <= 0)
    return false;
  else
    return true;
}

bool Socket::send(const std::string s) const
{
  int status = ::send(m_sock, s.c_str(), s.size(), MSG_NOSIGNAL);
  if (status == -1)
    return false;
  else
    return true;
}

int Socket::recv(std::string &s) const
{
  char buf[MAXRECV + 1];

  s = "";

  memset(buf, 0, MAXRECV + 1);

  int status = ::recv(m_sock, buf, MAXRECV, 0);

  if (status == -1)
  {
    // std::cout << "status == -1   errno == " << errno << "  in Socket::recv\n";
    return 0;
  }
  else if (status == 0)
  {
    return 0;
  }
  else
  {
    s = buf;
    return status;
  }
}

bool Socket::connect(const std::string host, const int port)
{
  if (!is_valid())
    return false;

  m_addr.sin_family = AF_INET;
  m_addr.sin_port = htons(port);

  int status = inet_pton(AF_INET, host.c_str(), &m_addr.sin_addr);

  if (errno == EAFNOSUPPORT)
    return false;

  status = ::connect(m_sock, (sockaddr *)&m_addr, sizeof(m_addr));

  if (status == 0)
    return true;
  else
    return false;
}

void Socket::set_non_blocking(const bool b)
{
  int opts;

  opts = fcntl(m_sock,
               F_GETFL);

  if (opts < 0)
  {
    return;
  }

  if (b)
    opts = (opts | O_NONBLOCK);
  else
    opts = (opts & ~O_NONBLOCK);

  fcntl(m_sock,
        F_SETFL, opts);
}

ServerSocket::ServerSocket(int port)
{
  if (!Socket::create())
    throw SocketException("Could not create server socket.");
  if (!Socket::bind(port))
    throw SocketException("Could not bind to port.");
  if (!Socket::listen())
    throw SocketException("Could not listen to socket.");
}

ServerSocket::~ServerSocket()
{
}

const ServerSocket &ServerSocket::operator<<(const std::string &s) const
{
  if (!Socket::send(s))
    throw SocketException("Could not write to socket.");
  return *this;
}

const ServerSocket &ServerSocket::operator>>(std::string &s) const
{
  if (!Socket::recv(s))
    throw SocketException("Could not read from socket.");
  return *this;
}

void ServerSocket::accept(ServerSocket &sock)
{
  if (!Socket::accept(sock))
    throw SocketException("Could not accept socket.");
}

ClientSocket::ClientSocket(std::string host, int port)
{
  if (!Socket::create())
  {
    throw SocketException("Could not create client socket.");
  }

  if (!Socket::connect(host, port))
  {
    throw SocketException("Could not bind to port.");
  }
}

const ClientSocket &ClientSocket::operator<<(const std::string &s) const
{
  if (!Socket::send(s))
  {
    throw SocketException("Could not write to socket.");
  }

  return *this;
}

const ClientSocket &ClientSocket::operator>>(std::string &s) const
{
  if (!Socket::recv(s))
  {
    throw SocketException("Could not read from socket.");
  }

  return *this;
}

#endif