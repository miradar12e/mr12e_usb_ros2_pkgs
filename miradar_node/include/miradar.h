#ifndef MIRADAR_H_
#define MIRADAR_H_
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

constexpr uint8_t CR = 0x0d;
constexpr uint8_t LF = 0x0a;
constexpr int COMM_RX_BYTE_UNIT = 64;

#define STDEF_INIT_COMM_PORT        "/dev/ttyACM0"

#define STDEF_INIT_RMAX             4.0
#define STDEF_INIT_RMIN             0.3
#define STDEF_INIT_RALM             1.0
#define STDEF_INIT_RDIV             64
#define STDEF_INIT_THDIV            45
#define STDEF_INIT_THMAX            44
#define STDEF_INIT_TXPWR            -5
#define STDEF_INIT_RXDBMIN          -74
#define STDEF_INIT_RXDBMAX          -30
#define STDEF_INIT_RXHPF            1
#define STDEF_INIT_RXPGA            1
#define STDEF_INIT_MEASITVL         200
#define STDEF_INIT_NUMPPIPLOT       1

#define STDEF_DISTANCE_MAX          200000
#define STDEF_ANGLE_MAX             80
#define STDEF_ANGLE_MIN             10
#define STDEF_DB_MAX                -10
#define STDEF_DB_MIN                -74


struct PPIData {
    int distance;
    int angle;
    int speed;
    int db;
};

struct MiRadarParam {
    explicit MiRadarParam()
        : minDistance((int)(STDEF_INIT_RMIN * 1000.0)),
          maxDistance((int)(STDEF_INIT_RMAX * 1000.0)),
          alarmDistance((int)(STDEF_INIT_RALM * 1000.0)),
          nDistance(STDEF_INIT_RDIV),
          nAngle(STDEF_INIT_THDIV),
          maxAngle(STDEF_INIT_THMAX),
          txPower(STDEF_INIT_TXPWR),
          minDb(STDEF_INIT_RXDBMIN),
          maxDb(STDEF_INIT_RXDBMAX),
          hpfGain(STDEF_INIT_RXHPF),
          pgaGain(STDEF_INIT_RXPGA),
          duration(STDEF_INIT_MEASITVL),
          nNumPpiPlot(STDEF_INIT_NUMPPIPLOT) {}

    int minDistance;
    int maxDistance;
    int alarmDistance;
    int nDistance;
    int nAngle;
    int maxAngle;
    int minAngle;
    int txPower;
    int minDb;
    int maxDb;
    int hpfGain;
    int pgaGain;
    int duration;
    int nNumPpiPlot;
};

class Serial {
   public:
    int fd;

    ~Serial() { close(fd); }

    int CommInit(std::string deviceFile) {
        fd = open(deviceFile.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        setupSerial();
        if (fd < 0) {
            return (-1);
        }
        return (fd);
    }

    void setupSerial() {
        struct termios tio;
        bzero((void*)&tio, (size_t)(sizeof(tio)));
        tio.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
        tio.c_iflag = IGNPAR;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tio.c_cc[VTIME] = 0;
        tio.c_cc[VMIN] = 1;

        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, &tio);
        fcntl(fd, F_SETFL, FNDELAY);
    }

    int CommTx(char* bpBuf, int nLen) {
        int nErr;
        nErr = write(fd, bpBuf, nLen);
        printf("TX %dbyte : %s", nLen, bpBuf);
        return (nErr);
    }
    int CommRx(char* bpBuf, int nBufSize, int nTmOut) {
        char* bp1;
        int nErr;
        int nRxLen = 0;
        int nNopCnt = 0;
        int nReqQuit = 0;
        memset(bpBuf, 0, nBufSize);
        bp1 = bpBuf;

        while (nReqQuit == 0) {
            nErr = -1;
            while (nErr < 0) {
                nErr = read(fd, bp1, COMM_RX_BYTE_UNIT);
                //------- received
                if (0 < nErr) {
                    nNopCnt = 0;
                    bp1 += nErr;
                    nRxLen += nErr;
                    if (nBufSize <= (nRxLen + COMM_RX_BYTE_UNIT)) {
                        nErr = -1;
                        nReqQuit = 1;
                        break;
                    }
                    continue;
                }
                //------- no received
                nNopCnt++;
                if(nRxLen==0) {
                    if(nTmOut<nNopCnt) {
                        nReqQuit = 1;
                        break;
                    }
                }
                //--------
                else if(10<nNopCnt) {
                    nReqQuit = 1;
                    break;
                }
                usleep(1000);
            }
        }

        return (nRxLen);
    }
};

std::vector<std::string> split(std::string str, char del) {
    int first = 0;
    int last = str.find_first_of(del);

    std::vector<std::string> result;

    while (first < (int)str.size()) {
        std::string subStr(str, first, last - first);

        result.push_back(subStr);

        first = last + 1;
        last = str.find_first_of(del, first);

        if (last == (int)(std::string::npos)) {
            last = str.size();
        }
    }

    return result;
}

char* toCharArray(std::string str) {
    char* chrArrC = reinterpret_cast<char*>(malloc(str.size() + 1));
    strcpy(chrArrC, str.c_str());
    return chrArrC;
}

class MiRadar {
public:
    std::vector<PPIData> ppiEntries;
    Serial comm;
    char sRxBuf[1024*1024];
    std::vector<uint8_t> map;
    int nDistance;
    int nAngle;
    int sensorState = 0;
    int prevState = 0;
    MiRadarParam radarParam;
    char sCmd[9] = {'A', ',', '0', ',', '1', ',', '0', 0x0d, 0x0a};

    explicit MiRadar() : sensorState(2) {
        radarParam.maxDistance = (int)(STDEF_INIT_RMAX * 1000.0);
        radarParam.minDistance = (int)(STDEF_INIT_RMIN * 1000.0);
        radarParam.nDistance = STDEF_INIT_RDIV;
        radarParam.alarmDistance = (int)(STDEF_INIT_RALM * 1000.0);
        radarParam.maxAngle = STDEF_INIT_THMAX;
        radarParam.nAngle = STDEF_INIT_THDIV;
        radarParam.txPower = STDEF_INIT_TXPWR;
        radarParam.hpfGain = STDEF_INIT_RXHPF;
        radarParam.pgaGain = STDEF_INIT_RXPGA; 
        radarParam.maxDb = STDEF_INIT_RXDBMAX;
        radarParam.minDb = STDEF_INIT_RXDBMIN;
        radarParam.duration = STDEF_INIT_MEASITVL;
        radarParam.nNumPpiPlot = STDEF_INIT_NUMPPIPLOT;
    }

    static double pixel2DB(int pix) {
        return static_cast<double>(pix) * 0.25 - 73.75;
    }

    void setSensorState(int state) {
        if (state <= 2 && state >= 0) {
            sCmd[4] = '0' + state;
            sensorState = state;
        }
    }

    void sendSensorMode() {
        //comm.CommTx(sCmd, sizeof(sCmd));                  // ST 2022_0704
        //comm.CommRx(sRxBuf, sizeof(sRxBuf), 100);
    }

    void stopCommunication() {
        std::string stopCommand = "B290_STOP";              // ST 2022_0704
        //std::string stopCommand = "A,0,0";                // original
        stopCommand.push_back(static_cast<char>(CR));
        stopCommand.push_back(static_cast<char>(LF));
        comm.CommTx(&stopCommand[0], static_cast<int>(strlen(stopCommand.c_str())));
        comm.CommRx(sRxBuf, sizeof(sRxBuf), 100);
    }

    int calcDuration(int numAngle, int numDistance) {
        float a = 0.031;

        // duration = 0.031 * number of Angle * number of distance
        int duration = static_cast<int>(static_cast<float>(numAngle) *
                                        static_cast<float>(numDistance) * a);
        int firstdigits = duration % 10;
        // make the time to 5 ms chunk
        duration = (firstdigits != 0) ? duration - firstdigits + 5 : duration;
        return duration;
    }

    void setParam() {
        validateParam(radarParam);
        std::string paramCommand = generateParamCommand(radarParam);
        //std::cout << paramCommand << std::endl;

        comm.CommTx(&paramCommand[0],
                    static_cast<int>(strlen(paramCommand.c_str())));
        comm.CommRx(sRxBuf, sizeof(sRxBuf), 100);
    }

    void validateDistance(int& minDistance, int& maxDistance) {
        constexpr int MAX_RANGE = STDEF_DISTANCE_MAX;
        // maximum min Distance = 256 * 25
        // this only applies when map state
        //if (sensorState == 2) {                       // 2022_1018
        //    minDistance = nDist * 25;                 // deleted for Patlite
        //    minDistance = (32 * 25 > minDistance) ? 32 * 25 : minDistance;
        //    minDistance = (minDistance > 256 * 25) ? 256 * 25 : minDistance;
        //}
        maxDistance = (maxDistance > MAX_RANGE) ? MAX_RANGE : maxDistance;
        maxDistance =
            (minDistance >= maxDistance) ? minDistance + 1000 : maxDistance;
    }

    void validateAlarmDistance(int& alarmDistance, int maxDistance) {
        constexpr int MAX_RANGE = STDEF_DISTANCE_MAX;
        alarmDistance = (alarmDistance < 0) ? 0 : alarmDistance;
        alarmDistance =
            (alarmDistance > maxDistance) ? maxDistance : alarmDistance;
        alarmDistance = (alarmDistance > MAX_RANGE) ? MAX_RANGE : alarmDistance;
    }

    void validateMaxAngle(int& maxAngle) {
        constexpr int MAX_ANGLE = STDEF_ANGLE_MAX;
        constexpr int MIN_ANGLE = STDEF_ANGLE_MIN;
        maxAngle = (maxAngle < MIN_ANGLE) ? MIN_ANGLE : maxAngle;
        maxAngle = (maxAngle > MAX_ANGLE) ? MAX_ANGLE : maxAngle;
    }

    void validateDB(int& minDB, int& maxDB) {
        constexpr int MAX_DB = STDEF_DB_MAX;
        constexpr int MIN_DB = STDEF_DB_MIN;
        minDB = (minDB > MAX_DB) ? MAX_DB : minDB;
        minDB = (minDB < MIN_DB) ? MIN_DB : minDB;
        maxDB = (maxDB > MAX_DB) ? MAX_DB : maxDB;
        maxDB = (maxDB < MIN_DB) ? MIN_DB + 1 : maxDB;
        minDB = (minDB >= maxDB) ? maxDB - 1 : minDB;
    }

    void validateTX(int& tx) {
        tx = (tx > 0) ? 0 : tx;
        tx = (tx < -10) ? -10 : tx;
    }

    void validateHPF(int& hpf) {
        hpf = (hpf < 0) ? 0 : hpf;
        hpf = (hpf > 2) ? 2 : hpf;
    }

    void validatePGA(int& pga) {
        pga = (pga < 0) ? 0 : pga;
        pga = (pga > 3) ? 3 : pga;
    }

    void validateNDistance(int& nDist) {
        nDist = (nDist < 32) ? 32 : nDist;
        nDist = static_cast<int>(pow(2, std::ceil(log2(nDist))));
        nDist = (nDist > 4096) ? 4096 : nDist;
    }

    void validateNAngle(int& nAng, int maxAngle) {
        nAng = (nAng > (maxAngle * 2 + 1)) ? maxAngle * 2 + 1 : nAng;
        nAng = (nAng < 11) ? 11 : nAng;
        nAng = ((maxAngle * 2) % ((nAng - 1)) != 0) ? maxAngle * 2 + 1 : nAng;
    }

    void validateDuration(int& nDuration) {
        if(nDuration<100) {
            nDuration = 100;
        }
        if(3000<nDuration) {
            nDuration = 3000;
        }
    }

    void validateParam(MiRadarParam& param) {
        validateNDistance(param.nDistance);
        validateDistance(param.minDistance, param.maxDistance);
        validateAlarmDistance(param.alarmDistance, param.maxDistance);
        validateMaxAngle(param.maxAngle);
        validateDB(param.minDb, param.maxDb);
        validateTX(param.txPower);
        validateHPF(param.hpfGain);
        validatePGA(param.pgaGain);
        validateNAngle(param.nAngle, param.maxAngle);
        validateDuration(param.duration);
        //param.duration = calcDuration(param.nAngle, param.nDistance);
    }

    std::string generateParamCommand(MiRadarParam& param) {
        std::string sensorStateStr =
            (sensorState == 1) ? "PPI" : "MAP";                                         // ST 2022_0704
        std::string paramCommand = "START_" + sensorStateStr + ",";                     // compatible ID with ST sample software
        paramCommand += std::to_string(param.maxDistance) + ",";
        paramCommand += std::to_string(param.minDistance) + ",";
        paramCommand += std::to_string(param.alarmDistance) + ",";
        paramCommand += std::to_string(param.nDistance) + ",";
        paramCommand += std::to_string(param.maxAngle) + ",";
        paramCommand += std::to_string(param.nAngle) + ",";
        paramCommand += std::to_string(param.txPower) + ",";
        paramCommand += std::to_string(param.hpfGain) + ",";
        paramCommand += std::to_string(param.pgaGain) + ",";
        paramCommand += std::to_string(param.minDb) + ",";
        paramCommand += std::to_string(param.duration);
        paramCommand.push_back(CR);
        paramCommand.push_back(LF);
        return paramCommand;
    }

    void setParam(MiRadarParam param) {
        static int nLocalPrevState = -1;
        if (nLocalPrevState==sensorState) {
            return;
        }
        nLocalPrevState = sensorState;
        setSensorState(sensorState);
        if(sensorState==0) {
            stopCommunication();
            return;
        }

        //---------------- ST 2022_0629 added average option
        //char sTxBuf[32];
        //int nLen;
        //sprintf(sTxBuf, "AVERAGE,4%c%c", 0x0d, 0x0a);
        //nLen = strlen(sTxBuf);
        //comm.CommTx(sTxBuf, nLen);
        //comm.CommRx(sRxBuf, sizeof(sRxBuf), 100);
        //usleep(100000);
        //--------------------------------------------------

        validateParam(param);

        std::string paramCommand = generateParamCommand(param);
        //std::cout << paramCommand << std::endl;

        char* commandCstr = toCharArray(paramCommand);
        comm.CommTx(commandCstr, static_cast<int>(strlen(commandCstr)));
        comm.CommRx(sRxBuf, sizeof(sRxBuf), 100);
    }

    void printParam(MiRadarParam param) {
        std::cout << "min distance : " << param.minDistance;
        std::cout << " max distance : " << param.maxDistance;
        std::cout << " duration : " << param.duration;
        std::cout << " min db : " << param.minDb;
        std::cout << " max db : " << param.maxDb;
        std::cout << " max angle : " << param.maxAngle;
        std::cout << " angle div : " << param.nAngle;
        std::cout << " distance div : " << param.nDistance << std::endl;
    }

    void setSerial(Serial& ser) { comm = ser; }

    char* getReceivedBuffer() { return sRxBuf; }

    void generatePPI(std::string& receivedBytes) {

        ppiEntries.clear();

        if ((int)(receivedBytes.find("BEGIN_PPI,1")) != -1 ||                                      // ST 2022_0704
            (int)(receivedBytes.find("BEGIN_PPI,0")) != -1) {
            std::vector<std::string> metadata = split(receivedBytes, ',');

            int entrynumbers = ((metadata.size()) - 1) / 4;
            if (entrynumbers > 1) {
                metadata.erase(metadata.begin());
                metadata.erase(metadata.begin());
            }
            entrynumbers = ((metadata.size()) - 1) / 4;
            if(entrynumbers!=8) {
                return;
            }
            for(int j = 0; j < entrynumbers; j++) {
                bool isNotEmpty = (std::stoi(metadata[4 * j + 0]) |
                                   std::stoi(metadata[4 * j + 1]) |
                                   std::stoi(metadata[4 * j + 2]) |
                                   std::stoi(metadata[4 * j + 3])) != 0;
                if (isNotEmpty) {
                    PPIData ppidata;
                    ppidata.distance = std::stoi(metadata[4 * j + 0]);
                    ppidata.angle    = std::stoi(metadata[4 * j + 1]);
                    ppidata.speed    = std::stoi(metadata[4 * j + 2]);
                    ppidata.db       = std::stoi(metadata[4 * j + 3]);
                    ppiEntries.push_back(ppidata);
                }
            }
        }
    }

    void generateMap(std::string& receivedBytes) {
        int nTop = (receivedBytes.find)("BEGIN_MAP,");
        if (0<=nTop) {
            //map.clear();
            int endIndex = receivedBytes.find(",END_MAP");
            if (endIndex != -1) {
                unsigned char *ubpBuf = (unsigned char *)(&receivedBytes[nTop]);
                int nPosR = 10;
                int nPosTh = 14;
                int nPosMap = 18;
                int nCnt = 0;
                int nLoop = 0;
                for(nLoop=0; ((nLoop<32)&&(nCnt<3)); nLoop++) {
                    if((ubpBuf[nLoop])==',') {
                        nCnt ++;
                        switch(nCnt) {
                            case 1:
                                nPosR = nLoop + 1;
                                break;
                            case 2:
                                nPosTh = nLoop + 1;
                                break;
                            case 3:
                                nPosMap = nLoop + 1;
                                break;
                            default:
                                break;
                        }
                    }
                }
                std::string mapStr = receivedBytes.substr((nTop+nPosMap), (endIndex-(nTop+nPosMap)));
                nDistance = atoi((const char *)(&ubpBuf[nPosR]));
                nAngle = atoi((const char *)(&ubpBuf[nPosTh]));
                //printf("distance=%d  angle=%d  R=%d  Th=%d  Map=%d rx=%d %d\n", nDistance, nAngle, nPosR, nPosTh, nPosMap, receivedBytes.size(), nTop);
                if ((nDistance * nAngle) == (endIndex - nPosMap)) {
                    map.clear();
                    std::copy(mapStr.begin(), mapStr.end(), std::back_inserter(map));
                } else {
                    //std::cout << "map is corrupt" << std::endl;
                    //std::cout << "map size is " << mapStr.size()
                    //          << " , requested size is " << (nDistance * nAngle) << std::endl;
                }
            }
        }
    }

    void run() {
        if (sensorState == 0) {
            // Halt mode
            return;
        }

        int size = comm.CommRx(sRxBuf, sizeof(sRxBuf), 0);
        std::string receivedBytes(sRxBuf, size);
        //std::cout << receivedBytes << std::endl;

        if (sensorState == 1) {
            // PPI Mode
            generatePPI(receivedBytes);
        } else if (sensorState == 2) {
            // Map Mode
            generateMap(receivedBytes);
        }
        prevState = sensorState;
    }
};

#endif
