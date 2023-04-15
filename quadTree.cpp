
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
              // vector<vector<vector<int>>> path; // { {ax, ay}, {bx, by} }
    bool operator==(const Path &p) const
    {
        return (p.ax == ax && p.ay == ay && p.bx == bx && p.by == by && p.dist == dist);
        // if (p.ax == ax && p.ay == ay && p.bx == bx && p.by == by && p.dist == dist)
        // {
        //     return true;
        // }
        // else
        // {
        //     return false;
        // }
    }

} Path;

int freeSquareCount = 0;
Square freeSquare[IMAGE_SIZE];

int occupiedSquareCount = 0;
Square occupiedSquares[IMAGE_SIZE];

//  Path *paths;

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
        std::cout << "this Path - ";
        thisPath.ax = Ax;
        std::cout << "Ax -";
        thisPath.ay = Ay;
        std::cout << "Ay -";
        thisPath.bx = Bx;
        std::cout << "Bx -";
        thisPath.by = By;
        std::cout << "By -";
        thisPath.dist = distance;
        std::cout << "distance -";

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

/*
pass in image coordinate eg. {90, 90} and will return vector containing the actual coordinates in the world
*/
int imageCoordToActualCoord(int value){ 
    return round(WORLD_SIZE*(1-(value/IMAGE_SIZE)));
}

/*
pass in actual coordinate eg. {90, 90} and will return vector containing the image coordinates for the image
*/
int actualCoordtoImageCoord(int value){
    return round(IMAGE_SIZE*(1-(value/WORLD_SIZE)));
    
}


void driveToPoints(vector<Path> paths)
{
    vector<vector<int> > a;
    vector<vector<int> > b;
    vector<int> dist;

    typedef struct Node
    {
        int x; //x loc
        int y; //y loc
        int dist; // distance to goal (needed for A*)
        vector<vector<int> > nodeLocations;
        vector<int> dToOtherNodes; // distance from this node to other nodes in the same order as nodes are put in
    } Node;


    vector<Node> listOfNodes;
    vector<Node> optPath;
    

    for (unsigned int i = 0; i < paths.size(); i++)
    {
        vector<int> aCombiner, bCombiner;

        aCombiner.clear();
        bCombiner.clear();
        aCombiner.push_back(paths.at(i).ax);
        aCombiner.push_back(paths.at(i).ay);

        bCombiner.push_back(paths.at(i).bx);
        bCombiner.push_back(paths.at(i).by);


        a.push_back(aCombiner);
        b.push_back(bCombiner);
        dist.push_back(paths.at(i).dist);
    }

    int startX = 550;
    int startY = 3500;
    int goalX = 3500;
    int goalY = 400;


    // Convert Start and end points into image coordinates
    
    
    Node startPoint;
    startPoint.x = actualCoordtoImageCoord(startX);
    startPoint.y = actualCoordtoImageCoord(startY);
    
    Node endPoint;
    endPoint.x = actualCoordtoImageCoord(goalX);
    endPoint.y = actualCoordtoImageCoord(goalY);
    
    startPoint.dist = sqrt((endPoint.x-startPoint.x)*(endPoint.x-startPoint.x)+(endPoint.y-startPoint.y)*(endPoint.y-startPoint.y));
    endPoint.dist = 0;
    // add start Node
    listOfNodes.push_back(startPoint);

    //Get all nodes and information
    for (int i = 0; i < paths.size(); i++){

        // if already in the list of nodes skip
        for (int v = 0; v < listOfNodes.size(); v++){
            if ((paths.at(i).ax == listOfNodes.at(v).x) && (paths.at(i).ay == listOfNodes.at(v).y)){
                break;
            }
        }

        Node newNode;
        newNode.x = paths.at(i).ax;
        newNode.y = paths.at(i).ay;
        newNode.dist = sqrt((endPoint.x-newNode.x)*(endPoint.x-newNode.x)+(endPoint.y-newNode.y)*(endPoint.y-newNode.y));

        // for each path connected to this node get next node location and distance to the node
        for (int j = 0; j< paths.size(); j++){

            // if path and node starting location is the same
            if ((paths.at(j).ax == newNode.x) && (paths.at(j).ay == newNode.y)){
                vector<int> newPathCoords;
                newPathCoords.push_back(paths.at(j).ax);
                newPathCoords.push_back(paths.at(j).ay);
                newNode.nodeLocations.push_back(newPathCoords);
                newNode.dToOtherNodes.push_back(paths.at(j).dist);
            }

        }

        listOfNodes.push_back(newNode);

    }

    // add end Node
    listOfNodes.push_back(endPoint);

    // print list of nodes
    printf("List of Nodes:\n");
    for (int i = 0; i < listOfNodes.size() -1 ; i++){
        printf("(%i, %i)\n", listOfNodes.at(i).x, listOfNodes.at(i).y);
    }

    //**************************************************************************************************
    // To-Do: calculate optimal path between start and goal
    // save path on optPath vector
    //***************************************************************************************************

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