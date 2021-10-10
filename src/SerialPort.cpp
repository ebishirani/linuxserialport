// System includes
#include <iostream>
#include <sstream>
#include <stdio.h> // Standard input/output definitions
#include <string.h>// String function definitions
#include <unistd.h>// UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <system_error>	// For throwing std::system_error
// Used for TCGETS2, which is required for custom baud rates
#include <sys/ioctl.h>
#include <cassert>
#include <asm/ioctls.h>
#include <asm/termbits.h>

#include "SerialPort.hpp"
#include "Exception.hpp"

#define    BOTHER 0010000

//---------------------------------------------------------------------------
linuxserialport::SerialPort::SerialPort()
{
    mEcho = false;
    mTimeoutMs = kDefaultTimeoutMs;
    mBaudRateType = BaudRateType::kStandard;
    mBaudRateStandard = kDefaultBaudRate;
    mReadBufferSizeB = kDefaultReadBufferSizeB;
    mReadBuffer.reserve(mReadBufferSizeB);
    mState = State::kClosed;
}
//---------------------------------------------------------------------------
linuxserialport::SerialPort::SerialPort(
        const std::string& device,
        BaudRate baudRate) :
        SerialPort()
{
    mDevice = device;
    mBaudRateType = BaudRateType::kStandard;
    mBaudRateStandard = baudRate;
}
//---------------------------------------------------------------------------
linuxserialport::SerialPort::SerialPort(
        const std::string& device,
        speed_t baudRate) :
        SerialPort()
{
    mDevice = device;
    mBaudRateType = BaudRateType::kCustom;
    mBaudRateCustom = baudRate;
}
//---------------------------------------------------------------------------
linuxserialport::SerialPort::~SerialPort()
{
    try
    {
        this->close();

        this->setTermios2(this->mOriginalSetting);
    }
    catch(...)
    {
        // We can't do anything about this!
        // But we don't want to throw within destructor, so swallow
    }
}
//---------------------------------------------------------------------------
void linuxserialport::SerialPort::setDevice(const std::string& device)
{
    mDevice = device;
    if(mState == State::kOpen)
        configureTermios();
}
//---------------------------------------------------------------------------
void linuxserialport::SerialPort::setBaudRate(BaudRate baudRate)
{
    std::cout << "standard called\n";
    mBaudRateType = BaudRateType::kStandard;
    mBaudRateStandard = baudRate;
    if(mState == State::kOpen)
        configureTermios();
}
//---------------------------------------------------------------------------
void linuxserialport::SerialPort::setBaudRate(speed_t baudRate)
{
    std::cout << " custom called\n";
    mBaudRateType = BaudRateType::kCustom;
    mBaudRateCustom = baudRate;
    if(mState == State::kOpen)
        configureTermios();
}
//---------------------------------------------------------------------------
void linuxserialport::SerialPort::open()
{
    if(mDevice.empty())
    {
        THROW_EXCEPT("Attempted to open file when file path has not been"
                     " assigned to.");
    }

    // O_RDONLY for read-only, O_WRONLY for write only, O_RDWR for both
    //read/write access
    // 3rd, optional parameter is mode_t mode
    mFileDesc = ::open(mDevice.c_str(), O_RDWR);

    // Check status
    if(mFileDesc == -1)
    {
        THROW_EXCEPT("Could not open device " + mDevice + ". Is the "
                     "device name correct and do you have read/write"
                     " permission?");
    }

    configureTermios();

    // std::cout << "COM port opened successfully." << std::endl;
    mState = State::kOpen;
}
//---------------------------------------------------------------------------
void linuxserialport::SerialPort::setEcho(bool value)
{
    mEcho = value;
    configureTermios();
}
//---------------------------------------------------------------------------
void linuxserialport::SerialPort::configureTermios()
{
    //================== CONFIGURE ==================//
    // termios tty = GetTermios();
    termios2 tty = getTermios2();
    //Save the main settings of the current serial port in order to reset it
    //after finishing working with the port.
    if (true == this->isNeedToHoldOriginalSetting)
    {
        this->mOriginalSetting = tty;
        this->isNeedToHoldOriginalSetting = false;
    }

    //================= (.c_cflag) ===============//
    // No parity bit is added to the output characters
    tty.c_cflag &= ~PARENB;
    // Only one stop-bit is used
    tty.c_cflag &= ~CSTOPB;
    // CSIZE is a mask for the number of bits per character
    tty.c_cflag &= ~CSIZE;
    // Set to 8 bits per character
    tty.c_cflag |= CS8;
    if (false == this->isEnableHWFlowControl)
    {
        // Disable hadrware flow control (RTS/CTS)
        tty.c_cflag &= ~CRTSCTS;
    }
    else
    {
        // Enable hadrware flow control (RTS/CTS)
        tty.c_cflag |= CRTSCTS;
    }

    // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_cflag |= CREAD | CLOCAL;
    //===================== BAUD RATE =================//

    // We used to use cfsetispeed() and cfsetospeed() with the B... macros,
    // but this didn't allow us to set custom baud rates. So now to support
    // both standard and custom baud rates lets just make everything "custom"
    // .This giant switch statement could be replaced with a map/lookup
    // in the future
    if (mBaudRateType == BaudRateType::kStandard)
    {
        tty.c_cflag &= ~CBAUD;
        tty.c_cflag |= CBAUDEX;
        switch(mBaudRateStandard)
        {
        case BaudRate::B_0:
            // cfsetispeed(&tty, B0);
            // cfsetospeed(&tty, B0);
            tty.c_ispeed = 0;
            tty.c_ospeed = 0;
            break;
        case BaudRate::B_50:
            // cfsetispeed(&tty, B50);
            // cfsetospeed(&tty, B50);
            tty.c_ispeed = 50;
            tty.c_ospeed = 50;
            break;
        case BaudRate::B_75:
            // cfsetispeed(&tty, B75);
            // cfsetospeed(&tty, B75);
            tty.c_ispeed = 75;
            tty.c_ospeed = 75;
            break;
        case BaudRate::B_110:
            // cfsetispeed(&tty, B110);
            // cfsetospeed(&tty, B110);
            tty.c_ispeed = 110;
            tty.c_ospeed = 110;
            break;
        case BaudRate::B_134:
            // cfsetispeed(&tty, B134);
            // cfsetospeed(&tty, B134);
            tty.c_ispeed = 134;
            tty.c_ospeed = 134;
            break;
        case BaudRate::B_150:
            // cfsetispeed(&tty, B150);
            // cfsetospeed(&tty, B150);
            tty.c_ispeed = 150;
            tty.c_ospeed = 150;
            break;
        case BaudRate::B_200:
            // cfsetispeed(&tty, B200);
            // cfsetospeed(&tty, B200);
            tty.c_ispeed = 200;
            tty.c_ospeed = 200;
            break;
        case BaudRate::B_300:
            // cfsetispeed(&tty, B300);
            // cfsetospeed(&tty, B300);
            tty.c_ispeed = 300;
            tty.c_ospeed = 300;
            break;
        case BaudRate::B_600:
            // cfsetispeed(&tty, B600);
            // cfsetospeed(&tty, B600);
            tty.c_ispeed = 600;
            tty.c_ospeed = 600;
            break;
        case BaudRate::B_1200:
            // cfsetispeed(&tty, B1200);
            // cfsetospeed(&tty, B1200);
            tty.c_ispeed = 1200;
            tty.c_ospeed = 1200;
            break;
        case BaudRate::B_1800:
            // cfsetispeed(&tty, B1800);
            // cfsetospeed(&tty, B1800);
            tty.c_ispeed = 1800;
            tty.c_ospeed = 1800;
            break;
        case BaudRate::B_2400:
            // cfsetispeed(&tty, B2400);
            // cfsetospeed(&tty, B2400);
            tty.c_ispeed = 2400;
            tty.c_ospeed = 2400;
            break;
        case BaudRate::B_4800:
            // cfsetispeed(&tty, B4800);
            // cfsetospeed(&tty, B4800);
            tty.c_ispeed = 4800;
            tty.c_ospeed = 4800;
            break;
        case BaudRate::B_9600:
            // cfsetispeed(&tty, B9600);
            // cfsetospeed(&tty, B9600);
            tty.c_ispeed = 9600;
            tty.c_ospeed = 9600;
            break;
        case BaudRate::B_19200:
            // cfsetispeed(&tty, B19200);
            // cfsetospeed(&tty, B19200);
            tty.c_ispeed = 19200;
            tty.c_ospeed = 19200;
            break;
        case BaudRate::B_38400:
            // cfsetispeed(&tty, B38400);
            // cfsetospeed(&tty, B38400);
            tty.c_ispeed = 38400;
            tty.c_ospeed = 38400;
            break;
        case BaudRate::B_57600:
            // cfsetispeed(&tty, B57600);
            // cfsetospeed(&tty, B57600);
            tty.c_ispeed = 57600;
            tty.c_ospeed = 57600;
            break;
        case BaudRate::B_115200:
            // cfsetispeed(&tty, B115200);
            // cfsetospeed(&tty, B115200);
            tty.c_ispeed = 115200;
            tty.c_ospeed = 115200;
            break;
        case BaudRate::B_230400:
            // cfsetispeed(&tty, B230400);
            // cfsetospeed(&tty, B230400);
            tty.c_ispeed = 230400;
            tty.c_ospeed = 230400;
            break;
        case BaudRate::B_460800:
            // cfsetispeed(&tty, B460800);
            // cfsetospeed(&tty, B460800);
            tty.c_ispeed = 460800;
            tty.c_ospeed = 460800;
            break;
        default:
            throw std::runtime_error(
                        std::string() +
                        "baudRate passed to " +
                        __PRETTY_FUNCTION__ +
                        " unrecognized.");
        }
    }
    // This does no different than STANDARD atm, but let's keep
    // them separate for now....
    else if (mBaudRateType == BaudRateType::kCustom)
    {
        tty.c_cflag &= ~CBAUD;
        tty.c_cflag |= CBAUDEX;
        // tty.c_cflag |= BOTHER;
        tty.c_ispeed = mBaudRateCustom;
        tty.c_ospeed = mBaudRateCustom;
    }
    else
    {
        // Should never get here, bug in this libraries code!
        assert(false);
    }

    //===================== (.c_oflag) =================//

    tty.c_oflag = 0;// No remapping, no delays
    tty.c_oflag &= ~OPOST;// Make raw

    //================= CONTROL CHARACTERS (.c_cc[]) ==================//

    // c_cc[VTIME] sets the inter-character timer, in units of 0.1s.

    // Only meaningful when port is set to non-canonical mode
    // VMIN = 0, VTIME = 0: No blocking, return immediately with what is
    //     available.

    // VMIN > 0, VTIME = 0: read() waits for VMIN bytes, could block
    //     indefinitely.

    // VMIN = 0, VTIME > 0: Block until any amount of data is available,
    //     OR timeout occurs.

    // VMIN > 0, VTIME > 0: Block until either VMIN characters have been
    //     received, or VTIME after first character has elapsed.

    // c_cc[WMIN] sets the number of characters to block (wait) for when
    //     read() is called.
    // Set to 0 if you don't want read to block. Only meaningful when port
    //     set to non-canonical mode.

    if(mTimeoutMs == -1)
    {
        // Always wait for at least one byte, this could
        // block indefinitely
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 1;
    }
    else if(mTimeoutMs == 0)
    {
        // Setting both to 0 will give a non-blocking read
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 0;
    }
    else if(mTimeoutMs > 0)
    {
        tty.c_cc[VTIME] = (cc_t)(mTimeoutMs/100);// 0.5 seconds read timeout
        tty.c_cc[VMIN] = 0;
    }

    //======================== (.c_iflag) ====================//

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);// Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|ICRNL|IGNCR|INLCR);
//    if (true == isRemoveCrLfActive)
//    {
//        tty.c_iflag |= IGNCR;
//        tty.c_iflag |= INLCR;
//    }
//    else
//    {
//        tty.c_iflag &= ~IGNCR;
//        tty.c_iflag &= ~INLCR;
//    }

    //========================= LOCAL MODES (c_lflag) =====================//
    // Canonical input is when read waits for EOL or EOF characters before
    //     returning. In non-canonical mode, the rate at which
    //     read() returns is instead controlled by c_cc[VMIN] and c_cc[VTIME]

    //Turn off canonical input, which is suitable for pass-through
    tty.c_lflag		&= ~ICANON;
    // Configure echo depending on echo_ boolean
    if(mEcho)
    {
        tty.c_lflag |= ECHO;
    }
    else
    {
        tty.c_lflag &= ~(ECHO);
    }
     // Turn off echo erase (echo erase only relevant if canonical input is
     //     active)
    tty.c_lflag		&= ~ECHOE;
    tty.c_lflag		&= ~ECHONL;
    // Disables recognition of INTR (interrupt), QUIT and SUSP (suspend)
    //     characters.
    tty.c_lflag		&= ~ISIG;

    // this->SetTermios(tty);
    this->setTermios2(tty);
}
//---------------------------------------------------------------------------
void linuxserialport::SerialPort::enableDisableStreamOutput(
        bool mustBeEnabled)
{
    if (this->isEnableHWFlowControl == mustBeEnabled)
    {
        this->isEnableHWFlowControl = !mustBeEnabled;
        this->configureTermios();
        if (true == mustBeEnabled)
        {
            ioctl(this->mFileDesc, TIOCMBIS, TIOCM_CTS);
        }
        else
        {
            ioctl(this->mFileDesc, TIOCMBIC, TIOCM_CTS);
        }
    }
}
//---------------------------------------------------------------------------
//void linuxserialport::SerialPort::removeCrlfFromOutput()
//{
//    this->isRemoveCrLfActive = true;
//    this->configureTermios();
//}
////---------------------------------------------------------------------------
//void linuxserialport::SerialPort::addCrlfToOutput()
//{
//    this->isRemoveCrLfActive = false;
//    this->configureTermios();
//}
//---------------------------------------------------------------------------
void linuxserialport::SerialPort::write(const std::string& data)
{

    if(mState != State::kOpen)
        THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ +
                     " called but state != OPEN. Please call Open() first.");

    if(mFileDesc < 0)
    {
        THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ +
                     "called but file descriptor < 0, indicating file has"
                     " not been opened.");
    }

    int writeResult = ::write(mFileDesc, data.c_str(), data.size());

    // Check status
    if (writeResult == -1)
    {
        throw std::system_error(EFAULT, std::system_category());
    }
}
//---------------------------------------------------------------------------
void linuxserialport::SerialPort::read(std::string& data)
{
    data.clear();

    if(mFileDesc == 0)
    {
        THROW_EXCEPT("Read() was called but file descriptor (fileDesc)"
                     " was 0, indicating file has not been opened.");
    }

    // Allocate memory for read buffer
    char buf [256];
    memset (&buf, '\0', sizeof buf);

    // Read from file
    // We provide the underlying raw array from the readBuffer_ vector to
    // this C api.
    // This will work because we do not delete/resize the vector while this
    // methodis called
    ssize_t n = ::read(mFileDesc, &mReadBuffer[0], mReadBufferSizeB);

    // Error Handling
    if(n < 0)
    {
        // Read was unsuccessful
        throw std::system_error(EFAULT, std::system_category());
    }

    if(n > 0)
    {
        buf[n] = '\0';
        //printf("%s\r\n", buf);
        data.append(buf);
        data = std::string(&mReadBuffer[0], n);
    }
}
//---------------------------------------------------------------------------
termios2 linuxserialport::SerialPort::getTermios2()
{
    struct termios2 term2;

    ioctl(mFileDesc, TCGETS2, &term2);

    return term2;
}
//---------------------------------------------------------------------------
void linuxserialport::SerialPort::setTermios2(termios2 tty)
{
    ioctl(mFileDesc, TCSETS2, &tty);
}
//---------------------------------------------------------------------------
void linuxserialport::SerialPort::close()
{
    if(mFileDesc != -1)
    {
        auto retVal = ::close(mFileDesc);
        if(retVal != 0)
        {
            THROW_EXCEPT("Tried to close serial port " +
                         mDevice + ", but close() failed.");
        }

        mFileDesc = -1;
    }

    mState = State::kClosed;
}
//---------------------------------------------------------------------------
void linuxserialport::SerialPort::setTimeout(int32_t timeout_ms)
{
    if(timeout_ms < -1)
        THROW_EXCEPT(std::string() + "timeout_ms provided to " +
                     __PRETTY_FUNCTION__ + " was < -1, which is invalid.");
    if(timeout_ms > 25500)
        THROW_EXCEPT(std::string() + "timeout_ms provided to " +
                     __PRETTY_FUNCTION__ +
                     " was > 25500, which is invalid.");
    if(mState == State::kOpen)
        THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ +
                     " called while state == OPEN.");
    mTimeoutMs = timeout_ms;
}
