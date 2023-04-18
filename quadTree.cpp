
#include <cmath>
#include <iostream>
#include <vector>
using std::vector;
#include <algorithm>
#include <array>
#include <string>

#include "eyebot++.h"

#define WORLD_SIZE 4000
#define IMAGE_SIZE 128

BYTE *image;
char *fileName = "diagonal.pbm";
int arr[IMAGE_SIZE][IMAGE_SIZE];
// vector<vector<vector<int>>> paths;
#define LINE_MAX 255
#define SPEED 300

void read_pbm_header(FILE *file, int *width, int *height)
{
    char line[LINE_MAX];
    char word[LINE_MAX];
    char *next;
    int read;
    int step = 0;
    while (1)
    {
        if (!fgets(line, LINE_MAX, file))
        {
            fprintf(stderr, "Error: End of file\n");
            exit(EXIT_FAILURE);
        }
        next = line;
        if (next[0] == '#')
            continue;
        if (step == 0)
        {
            int count = sscanf(next, "%s%n", word, &read);
            if (count == EOF)
                continue;
            next += read;
            if (strcmp(word, "P1") != 0 && strcmp(word, "p1") != 0)
            {
                fprintf(stderr, "Error: Bad magic number\n");
                exit(EXIT_FAILURE);
            }
            step = 1;
        }
        if (step == 1)
        {
            int count = sscanf(next, "%d%n", width, &read);
            if (count == EOF)
                continue;
            next += read;
            step = 2;
        }
        if (step == 2)
        {
            int count = sscanf(next, "%d%n", height, &read);
            if (count == EOF)
                continue;
            next += read;
            return;
        }
    }
}

void read_pbm_data(FILE *file, int width, int height, BYTE *img)
{
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            char c = ' ';
            while (c == ' ' || c == '\n' || c == '\r')
                c = fgetc(file);

            if (c != '0' && c != '1')
            {
                fprintf(stderr, "Bad character: %c\n", c);
                exit(0);
            }
            arr[x][y] = c;
            *img = c - '0';
            img++;
        }
        printf("\n");
    }
}

void read_pbm(char *filename, BYTE **img)
{
    int width;
    int height;
    FILE *file = fopen(filename, "r");
    if (!file)
    {
        fprintf(stderr, "Error: Cannot open the input file\n");
        exit(EXIT_FAILURE);
    }
    read_pbm_header(file, &width, &height);
    *img = (BYTE *)malloc(width * height * sizeof(BYTE));
    read_pbm_data(file, width, height, *img);
    fclose(file);
}

typedef struct Square
{
    // What information do we need here?
    // Top left coordinates?
    // Size of Square?
    // Occupied or not?
    int size;
    int locX;
    int locY;
    int occupied; // 1 = occupied and 0 means empty

} Square;

typedef struct Path
{
    int ax; // startpoint
    int ay;
    int bx; // endpoint
    int by;
    int dist; // dist

    bool operator==(const Path &p) const
    {
        return (p.ax == ax && p.ay == ay && p.bx == bx && p.by == by && p.dist == dist);
    }

} Path;

typedef struct Node
{
    int idx;  // node index
    int x;    // x loc
    int y;    // y loc
    int dist; // distance to goal (needed for A*)
    // vector<vector<int>> nodeLocations; // ? locations of all the connected nodes
    // vector<int> dToOtherNodes;         // ? distance from this node to other nodes in the same order as nodes are put in
    //  nodeLocations[i] is connected to dToOtherNodes[i]
} Node;

int freeSquareCount = 0;
Square freeSquare[IMAGE_SIZE];

int occupiedSquareCount = 0;
Square occupiedSquares[IMAGE_SIZE];

/*
Performs recursive quadtree division of image
and stores the free and occupied squares.
*/
void QuadTree(int x, int y, int size)
{
    bool allFree = true;
    bool allOccupied = true;

    for (int i = x; i < x + size; i++)
        for (int j = y; j < y + size; j++)
            if (image[i * IMAGE_SIZE + j])
                allFree = false; // at least 1 occ.
            else
                allOccupied = false; // at least 1 free

    Square square;
    square.size = size;
    square.locX = x + round(size / 2);
    square.locY = y + round(size / 2);
    if (allFree == true)
        square.occupied = 0;
    else if (allOccupied == true)
        square.occupied = 1;

    if (allFree)
    {
        // Experiment 1 print centre points
        printf("Centre point: %d %d %d\n", square.locX, square.locY, size);

        // Store the free squares
        freeSquare[freeSquareCount] = square;
        freeSquareCount++;

        // Draw the square (with slight margins)
        LCDArea(y + 1, x + 1, y + size - 1, x + size - 2, GREEN, 0);

        // Draw a circle in all unoccupied areas
        LCDCircle(square.locY, square.locX, square.size / 5, GREEN, true);
    }
    else if (allOccupied)
    {
        // Add an occupied square
        occupiedSquares[occupiedSquareCount] = square;
        occupiedSquareCount++;

        // Draw the sqaure with slight margins
        LCDArea(y + 1, x + 1, y + size - 1, x + size - 2, RED, 0);
    }
    // recusive calls
    if (!allFree && !allOccupied && size > 1)
    { // size == 1 stops recursion
        int s2 = size / 2;
        QuadTree(x, y, s2);
        QuadTree(x + s2, y, s2);
        QuadTree(x, y + s2, s2);
        QuadTree(x + s2, y + s2, s2);
    }
}

/*
Prints and displays all of the collision free paths
between all pairs of free squares
Note uses variable names as per lecture slides
*/

void collisionFreePaths(vector<Path> &paths, int pathCount)
{
    int Rx, Ry, Sx, Sy, Tx, Ty, Ux, Uy, Ax, Ay, Bx, By;

    for (int i = 0; i < 2; i++)
    { // cheated this part next for loop should start at 0 not 2
        int j = i + 1;
        Ax = freeSquare[i].locX;
        Ay = freeSquare[i].locY;
        Bx = freeSquare[j].locX;
        By = freeSquare[j].locY;

        LCDLine(Ay, Ax, By, Bx, BLUE); // Draw it on screen

        int distance = sqrt(pow(Ax - Bx, 2) + pow(Ay - By, 2));

        printf("Distance From (%i, %i) -> (%i, %i): %i\n", Ax, Ay, Bx, By, distance);

        Path thisPath;
        thisPath.ax = Ax;
        thisPath.ay = Ay;
        thisPath.bx = Bx;
        thisPath.by = By;
        thisPath.dist = distance;

        paths.push_back(thisPath);
        printf("Distance From (%i, %i) -> (%i, %i): %i\n", Ax, Ay, Bx, By, distance);
        pathCount++;
    }

    for (int i = 2; i < freeSquareCount; i++)
    {
        for (int j = i + 1; j < freeSquareCount; j++)
        {
            // for all pairs of free squares

            bool overOccupiedSquare = false;

            // Check all occupied squares to see if any intersect the path between two squares

            for (int k = 0; k < occupiedSquareCount; k++)
            {
                if (freeSquare[i].size <= 8 || freeSquare[j].size <= 8)
                {
                    break;
                }
                int negativeFs = 0;
                int positiveFs = 0;

                // TODO count negative and positive Fs as per algorithm
                Rx = occupiedSquares[k].locX - occupiedSquares[k].size / 2;
                Ry = occupiedSquares[k].locY - occupiedSquares[k].size / 2;

                Sx = occupiedSquares[k].locX + occupiedSquares[k].size / 2;
                Sy = occupiedSquares[k].locY - occupiedSquares[k].size / 2;

                Tx = occupiedSquares[k].locX - occupiedSquares[k].size / 2;
                Ty = occupiedSquares[k].locY + occupiedSquares[k].size / 2;

                Ux = occupiedSquares[k].locX + occupiedSquares[k].size / 2;
                Uy = occupiedSquares[k].locY + occupiedSquares[k].size / 2;

                Ax = freeSquare[i].locX;
                Ay = freeSquare[i].locY;
                Bx = freeSquare[j].locX;
                By = freeSquare[j].locY;
                int distance = sqrt(pow(Ax - Bx, 2) + pow(Ay - By, 2));
                if (distance > 10)
                {
                    // printf("Distance too big\n");
                    break;
                }

                double f1 = (By - Ay) * Rx + (Ax - Bx) * Ry + (Bx * Ay - Ax * By);
                double f2 = (By - Ay) * Sx + (Ax - Bx) * Sy + (Bx * Ay - Ax * By);
                double f3 = (By - Ay) * Tx + (Ax - Bx) * Ty + (Bx * Ay - Ax * By);
                double f4 = (By - Ay) * Ux + (Ax - Bx) * Uy + (Bx * Ay - Ax * By);

                if (f1 < 0)
                {
                    negativeFs++;
                }
                else if (f1 != 0)
                {
                    positiveFs++;
                }
                if (f2 < 0)
                {
                    negativeFs++;
                }
                else if (f2 != 0)
                {
                    positiveFs++;
                }
                if (f3 < 0)
                {
                    negativeFs++;
                }
                else if (f3 != 0)
                {
                    positiveFs++;
                }
                if (f4 < 0)
                {
                    negativeFs++;
                }
                else if (f4 != 0)
                {
                    positiveFs++;
                }

                if (negativeFs == 4 || positiveFs == 4)
                {
                    // All ponts above or below line
                    // no intersection, check the next occupied square
                    continue;
                }
                else
                {
                    // Get variables as needed for formula

                    // formula as per lecture slides
                    printf("not negativeFs or positiveFs");

                    overOccupiedSquare = !((Ax > Sx && Bx > Sx) || (Ax < Tx && Bx < Tx) || (Ay > Sy && By > Sy) || (Ay < Ty && By < Ty));
                    if (!overOccupiedSquare)
                    {
                        printf(" --- passed\n");
                    }

                    if (overOccupiedSquare)
                    {
                        // this is not a collision free path
                        printf(" --- broke\n");
                        break;
                    }
                }
            }

            if (!overOccupiedSquare)
            {
                // a collision free path can be found so draw it

                LCDLine(Ay, Ax, By, Bx, BLUE); // Draw it on screen

                int distance = sqrt(pow(Ax - Bx, 2) + pow(Ay - By, 2));

                Path thisPath;
                thisPath.ax = Ax;
                thisPath.ay = Ay;
                thisPath.bx = Bx;
                thisPath.by = By;
                thisPath.dist = distance;

                if (!(std::find(paths.begin(), paths.end(), thisPath) != paths.end()))
                {
                    paths.push_back(thisPath);
                    printf("Distance From (%i, %i) -> (%i, %i): %i\n", Ax, Ay, Bx, By, distance);
                    pathCount++;
                }
            }
            else
            {
                printf("Did not print\n");
            }
        }
    }
}

vector<Node> aStar(vector<Node> listOfNodes, vector<Path> paths)
{
    vector<Node> bestRoute;
    typedef struct nodeInfo
    {
        vector<int> nodeIndices;
        int dist;   // sum of path lengths from start to last node in the list
        int length; // dist + node value

    } nodeInfo;

    vector<nodeInfo> visitedNodes;
    int currNodeIdx = 0;
    bool initializeNodes = true;
    KEYWait(KEY3);

    // while not at the goal
    while ((listOfNodes.at(currNodeIdx).x != listOfNodes.at(listOfNodes.size() - 1).x) && (listOfNodes.at(currNodeIdx).y != listOfNodes.at(listOfNodes.size() - 1).y))
    {
        // find shortest path
        for (size_t p = 0; p < paths.size(); p++) // loop over paths
        {
            // find all paths which start from the current node in the vector of paths
            // calculate the path lengths to each node and save them in the vector of visited nodes
            if (initializeNodes)
            {
                if (listOfNodes.at(currNodeIdx).x == paths.at(p).ax && listOfNodes.at(currNodeIdx).y == paths.at(p).ay)
                {

                    nodeInfo currentNode;
                    for (size_t n = 0; n < listOfNodes.size(); n++)
                    {
                        if (listOfNodes.at(n).x == paths.at(p).bx && listOfNodes.at(n).y == paths.at(p).by)
                        {
                            currentNode.nodeIndices.push_back(n);
                        }
                    }
                    //  index in vector listOfNodes where listOfNodes.x = paths.at(i).bx && listOfNodes.y = paths.at(i).by
                    // if (currentNode.dist)
                    currentNode.dist = paths.at(p).dist;
                    currentNode.length = currentNode.dist + listOfNodes.at(currNodeIdx).dist;
                    visitedNodes.push_back(currentNode);
                }
                initializeNodes = false;
            }
            else
            {
                if (listOfNodes.at(currNodeIdx).x == paths.at(p).ax && listOfNodes.at(currNodeIdx).y == paths.at(p).ay)
                {
                    nodeInfo currentNode;
                    for (size_t v = 0; v < visitedNodes.size(); v++)
                    {
                        if (visitedNodes.at(v).nodeIndices.back() == currNodeIdx)
                        {
                            for (size_t n = 0; n < listOfNodes.size(); n++)
                            {
                                if (listOfNodes.at(n).x == paths.at(p).bx && listOfNodes.at(n).y == paths.at(p).by)
                                {
                                    visitedNodes.at(v).nodeIndices.push_back(n);
                                    visitedNodes.at(v).dist += paths.at(p).dist;
                                    visitedNodes.at(v).length = visitedNodes.at(v).dist + listOfNodes.at(currNodeIdx).dist;
                                }
                            }
                        }
                    }
                    //  index in vector listOfNodes where listOfNodes.x = paths.at(i).bx && listOfNodes.y = paths.at(i).by
                    // if (currentNode.dist)
                    currentNode.dist = paths.at(p).dist;
                    currentNode.length = paths.at(p).dist + listOfNodes.at(currNodeIdx).dist;
                    visitedNodes.push_back(currentNode);

                    std::cout << "Visited nodes { ";
                    for (size_t i = 0; i < currentNode.nodeIndices.size(); i++)
                    {
                        std::cout << currentNode.nodeIndices.at(i) << " ";
                    }
                    std::cout << "}" << std::endl;
                    std::cout << " -  dist " << currentNode.dist << ", length " << currentNode.length << std::endl;
                }
            }

            // select the current node as the node with minimal length
            // the current node will be further explored in the next iteration
            int shortestPath = 10000;
            for (size_t i = 0; i < visitedNodes.size(); i++)
            {
                if (visitedNodes.at(i).length < shortestPath)
                {
                    shortestPath = visitedNodes.at(i).length;
                    currNodeIdx = visitedNodes.at(i).nodeIndices.back();

                    if (currNodeIdx == listOfNodes.size() - 1)
                    {
                        // save best path
                        for (size_t w = 0; w < visitedNodes.at(i).nodeIndices.size(); w++)
                        {
                            Node waypoint;
                            waypoint.idx = visitedNodes.at(i).nodeIndices.at(w);
                            waypoint.x = listOfNodes.at(waypoint.idx).x;
                            waypoint.y = listOfNodes.at(waypoint.idx).y;
                            LCDCircle(waypoint.y, waypoint.x, 10, BLUE, true);

                            // find pathlength of waypoint
                            for (size_t p = 0; p < paths.size(); p++)
                            {
                                if (waypoint.x == paths.at(p).ax && waypoint.y == paths.at(p).ay)
                                {
                                    waypoint.dist = paths.at(p).dist;
                                }
                            }
                            bestRoute.push_back(waypoint);
                        }
                        break;
                    }
                }
            }
        }
    }

    return bestRoute;
}

void driveToPoints(vector<Path> paths)
{
    int startX = 550;
    int startY = 3500;
    int goalX = 3500;
    int goalY = 400;
    int imageStartX = IMAGE_SIZE * ((startX / WORLD_SIZE));
    int imageStartY = IMAGE_SIZE * (1 - (startY / WORLD_SIZE));
    int imageEndX = 128 * (1 - (goalX / 4000));
    int imageEndY = IMAGE_SIZE * (1 - (goalY / WORLD_SIZE));

    printf("Image start point, = (%i, %i)\n", imageStartX, imageStartY);
    printf("Image end point, = (%i, %i)\n\n", imageEndX, imageEndY);
    LCDCircle(imageStartY, imageStartX, 10, BLUE, true);
    LCDCircle(imageEndY, imageEndX, 10, BLUE, true);
    vector<vector<int> > a;
    vector<vector<int> > b;
    vector<int> dist;

    int nodeIdx = 0;
    vector<Node> listOfNodes;
    bool inList;

    // Get all nodes and information
    for (unsigned int i = 0; i < paths.size(); i++)
    {
        inList = false;

        // if already in the list of nodes skip
        for (unsigned int v = 0; v < listOfNodes.size(); v++)
        {
            if ((paths.at(i).ax == listOfNodes.at(v).x) && (paths.at(i).ay == listOfNodes.at(v).y))
            {
                inList = true;
            }
        }

        if (!inList)
        {
            Node newNode;
            newNode.idx = nodeIdx;
            newNode.x = paths.at(i).ax;
            newNode.y = paths.at(i).ay;
            newNode.dist = sqrt(pow(imageEndX - newNode.x, 2) + pow(imageEndY - newNode.y, 2));
            listOfNodes.push_back(newNode);
            nodeIdx++;
        }
    }

    // add end Node
    // listOfNodes.push_back(endPoint);

    // print list of nodes
    printf("List of Nodes, List.size = %lu\n", listOfNodes.size());
    // printf("Num, (x, y),  distToEnd\n\n", listOfNodes.size());

    for (unsigned int i = 0; i < listOfNodes.size(); i++)
    {
        listOfNodes.at(i).idx = i;
        printf("Num %i, (%i, %i), dist = %i \n", i, listOfNodes.at(i).x, listOfNodes.at(i).y, listOfNodes.at(i).dist);
    }

    //*******************************************************************
    // calculate optimal path between start and goal
    vector<Node> optPath = aStar(listOfNodes, paths); 
    //*******************************************************************

    printf("\nOptimal Path, Path size = %lu\n", optPath.size());
    for (unsigned int i = 0; i < optPath.size(); i++)
    {
        printf("(%i, %i)\n", optPath.at(i).x, optPath.at(i).y);
    }

    // drive along optimal path
    for (unsigned int i = 0; i < optPath.size() - 1; i++)
    {
        // calculate heading to next node
        int x, y, phi;
        VWGetPosition(&x, &y, &phi);
        int theta = round(atan2(optPath.at(i).x - optPath.at(i + 1).x, optPath.at(i).y - optPath.at(i + 1).y) * 180.0 / M_PI);
        if (theta > 180.0)
            theta -= 360;

        // turn towards node
        int diff = round(theta - phi);
        VWTurn(diff, 90); // check if direction is right
        VWWait();

        // drive to node
        VWStraight(optPath.at(i + 1).dist, SPEED);
    }
}

void printfImage(BYTE img)
{
    for (int i = 0; i < IMAGE_SIZE; i++)
    {
        for (int j = 0; j < IMAGE_SIZE; j++)
        {
            // Both %d and %X works
            printf("%d", image[i * IMAGE_SIZE + j]);
        }
        printf("\n");
    }
    printf("IMAGE PRINTED \n");
}

int main()
{
    // Read the file;
    read_pbm(fileName, &image);
    LCDImageStart(0, 0, IMAGE_SIZE, IMAGE_SIZE);
    LCDImageBinary(image); // this has to be binary

    int endSim = 0; // boolean to end sim 0 = false, 1 = true
    LCDMenu("QUADTREE", "PATHS", "DRIVE", "EXIT");

    int pathCount = 0;
    vector<Path> paths;
    VWSetPosition(400, 3500, 0); // eventually change x and y

    do
    {
        switch (KEYRead())
        {
        case KEY1:
            printf("\nExperiment 1\n---\n");
            QuadTree(0, 0, 128);
            // prints the image using printf()
            printfImage(*image);
            break;
        case KEY2:
            printf("\nExperiment 2 and 3\n---\n");
            collisionFreePaths(paths, pathCount);
            break;
        case KEY3:
            printf("\nExperiment 4\n---\n");
            driveToPoints(paths);
            break;
        case KEY4:
            endSim = 1;
            break;
        }
    } while (!endSim);
}