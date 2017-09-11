#ifndef PROJECT_SIDEWALKEDGE_H
#define PROJECT_SIDEWALKEDGE_H

//opencv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"
#include <sensor_msgs/Image.h>

#include "../RecognizeSidewalkParams.h"
#include "../pavement_to_cloud/pavement_to_cloud.h"

#define EDGE_MARKER_TYPE 8
#define EDGE_MARKER_SHIFT 0

struct LineStructure{
    cv::Point start;
    cv::Point end;
    cv::Point startOld;
    cv::Point endOld;
    lineEquation myLineEquation;
    lineEquation myLineEquationOld;
    bool valid = true;
};
class Edge{
private:
    std::vector<LineStructure> edge;
public:
    Edge();
    ~Edge();
    void computeLineEquation();
    void new_line();
    void drawEdge(cv::Mat *img, const cv::Scalar& color, RecognizeSidewalkParams *params);
    std::vector<LineStructure> *getEdge();
};

class SidewalkEdge{
private:
    Edge edgeRaw;//purple
    Edge edgeValid;//green
    Edge edgeFix;//yellow
    Edge edgeOld;
    std::vector<cv::Point2f> oldPoints;
    std::vector<cv::Point2f> newPoints;
    cv::Mat oldImg;
    cv::Mat newImg;
    std::vector<int> sidewalkArea;
    std::vector<double> medianSlope;
    double medianSlopeFrame = 0;
    int sidewalkAreaFrame;
    bool glitchedFrameVal = false;
    bool glitchedFrameSlopeVal = false;
    //TODO function which determines which edges are valid in consideration with old frames
    //todo add line queue for line which replaces invalid lines - display with diferent color (yellow)
    //todo test contours for edge
    //todo optical flow will help us determine if we are turning and whether we are perpendicular to sidewalk
    //todo zda sa, ze ked je presvetleny obraz, tak segmentacia je dobra aspon pri robotovy, budeme teda pri rozpoznani hran toto brat do uvahy (budeme pocitat iba so spodkom obrazu).
    //todo moja detekcia bodov okraja chodnika bude dodavat body do optickeho toku a ten bude uz dalej udrziavat chodnik. Zdetekujeme celkovy pohyb kamery, ten odfiltrujeme a zvyskovy pohyb bude sposobeny fluktuaciami v segmentacii obrazu.
    //todo nebude buffer edge ako je teraz, ale staré detekované body posunuté optickým tokom na novom obraze a staré detekované body. Stare detekovane body budu zjednotenim bodov Valid a Fix (nakoniec Fix bude uz fuzia Valid a opravenych bodov)
    //todo ak viacej framov za sebou daný úsek okraja nebude detekovaný správne, tak sa nezobrazí ani na Fix okraji.
    //todo nastava problem ze zapis okraja po ciarach nie je vhodny pre spracovanie cez opticky tok
    //todo spraviť kalibráciu optického toku (prahových hodnôt kedy brať detekovaný bod ako zlý) pre každý detekovaný riadok samostatne, z nejakej mediánovej hodnoty sa vytvorí pre každý konštanta prahu. (spraviť na to funkciu, čo to spraví automaticky).
    //todo Do vysledneho vektora bodov, by mali ist uz iba novo detekovane body, aby sme nezakreslovali stare body viac krat

    //TODO dolezite - z rovnic priamok spravime median a potom tie ciary, ktore budu vybiehat z medianu nahradime ciarou s rovnicou priamky median
    //TODO dolezite - zahadzovat framy, ktore maju median rovnic priamok iny ako predosle framy
    //TODO median slope prerobit na median uhol pretoze slope je exp funkcia a to je zle

    //todo reenable offset - do it after all validation and fixation and put it into new vector
public:
    bool invalidFrame = false;
    bool perpendicularSidewalk = false;
    std::vector<cv::Point> validPoints;
    SidewalkEdge();
    ~SidewalkEdge();
    void validateEdge(RecognizeSidewalkParams *params, int pavementCenter);
    void fillLineWithPoints(std::vector<cv::Point> *outFilledPointsEdge, lineEquation lineEquationC, cv::Point lineCmStart, cv::Point lineCmEnd);
    void rawLinesToPoints();
    void fixEdge();
    void computeLineEquation();
    int computeOpticalFlow(cv::Mat *gray, cv::Mat *prevGray, std::vector<cv::Point2f> *points, std::vector<cv::Point2f> *prevPoints);
    void new_line();
    void clearEdge();
    void setImgToDetect(cv::Mat *img);
    bool isOpening(int imgCols, int startPoint, int endPoint, int sideOffset);
    void getSidewalkArea(cv::Mat *img, RecognizeSidewalkParams *params);
    bool glitchedFrame();
    bool glitchedFrameSlope();
    void detectFrameGlitch(RecognizeSidewalkParams *params);
    int computeSlopeMedian(RecognizeSidewalkParams *params);
    void slopeValidate(RecognizeSidewalkParams *params);
    bool isSidewalkPerpendicular(RecognizeSidewalkParams *params);

    void drawEdgeRaw(cv::Mat *img, RecognizeSidewalkParams *params);
    void drawEdgeValid(cv::Mat *img, RecognizeSidewalkParams *params);
    void drawEdgeFix(cv::Mat *img, RecognizeSidewalkParams *params);
    void drawAllEdges(cv::Mat *img, RecognizeSidewalkParams *params);
    void drawDetectedPoints(cv::Mat *img, RecognizeSidewalkParams *params);

    std::vector<LineStructure> *getEdgeRaw();
    std::vector<LineStructure> *getEdgeValid();
    std::vector<LineStructure> *getEdgeFix();
};

struct SidewalkEdges
{
    //todo turn to class
    SidewalkEdge left;
    SidewalkEdge right;
};

bool notPavement(int startPoint, int endPoint, int pavementCenter, int sideOffset);

#endif //PROJECT_SIDEWALKEDGE_H
