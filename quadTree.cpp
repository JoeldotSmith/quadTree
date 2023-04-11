
#include <cmath>
#include <iostream>
#include <vector>
using std::vector; 
#include <array>

#include "eyebot++.h"

#define WORLD_SIZE 4000
#define IMAGE_SIZE 128

BYTE *image;
char *fileName = "diagonal.pbm";
int arr[IMAGE_SIZE][IMAGE_SIZE];
#define LINE_MAX 255

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
    // What info do we need here
    // Array of points?

    vector<vector<vector<int> > > path; // { {ax, ay}, {bx, by} }

} Path;

int freeSquareCount = 0;
Square freeSquare[IMAGE_SIZE];

int occupiedSquareCount = 0;
Square occupiedSquares[IMAGE_SIZE];

int pathCount = 0;
Path *paths;

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
    square.locX = x+size/2;
    square.locY = y+size/2;
    if(allFree==true)
        square.occupied = 0;
    else if (allOccupied==true) 
        square.occupied = 1;

    if (allFree)
    {
        // Experiment 1 print centre points
        printf("Centre point: %d %d %d\n", x + (int)size / 2, y + (int)size / 2, size);

        // Store the free squares
        freeSquare[freeSquareCount] = square; // TODO
        freeSquareCount++;

        // Draw the square (with slight margins)
        LCDArea(y + 1, x + 1, y + size - 1, x + size - 2, GREEN, 0);
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

void collisionFreePaths()
{
    int Rx, Ry, Sx, Sy, Tx, Ty, Ux, Uy, Ax, Ay, Bx, By;

    for (int i = 0; i < 2; i++){ // cheated this part next for loop should start at 0 not 2
        int j = i+1;
            Ax = freeSquare[i].locX;
            Ay = freeSquare[i].locY;
            Bx = freeSquare[j].locX;
            By = freeSquare[j].locY;


            LCDLine(Ay, Ax, By, Bx, BLUE); // Draw it on screen

            int distance = sqrt(pow(Ax-Bx, 2)+pow(Ay-By, 2));

            printf("Distance From (%i, %i) -> (%i, %i): %i\n", Ax, Ay, Bx, By, distance);
            
            
            vector<vector<int> > path2d;
            vector<int> path1da;
            vector<int> path1db;
            path1da.push_back(Ax);
            path1da.push_back(Ay);
            path1db.push_back(Bx);
            path1db.push_back(By);

            path2d.push_back(path1da);
            path2d.push_back(path1db);

            //paths->path.push_back(path2d);

            pathCount++;

            
            
            

            // TODO store these path  
        
        
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
                if (freeSquare[i].size <= 8 || freeSquare[j].size <= 8){
                    break;

                }
                int negativeFs = 0;
                int positiveFs = 0;

                // TODO count negative and positive Fs as per algorithm
                Rx = occupiedSquares[k].locX-occupiedSquares[k].size/2;
                Ry = occupiedSquares[k].locY-occupiedSquares[k].size/2;

                Sx = occupiedSquares[k].locX+occupiedSquares[k].size/2;
                Sy = occupiedSquares[k].locY-occupiedSquares[k].size/2;

                Tx = occupiedSquares[k].locX-occupiedSquares[k].size/2;
                Ty = occupiedSquares[k].locY+occupiedSquares[k].size/2;

                Ux = occupiedSquares[k].locX+occupiedSquares[k].size/2;
                Uy = occupiedSquares[k].locY+occupiedSquares[k].size/2;

                Ax = freeSquare[i].locX;
                Ay = freeSquare[i].locY;
                Bx = freeSquare[j].locX;
                By = freeSquare[j].locY;
                int distance = sqrt(pow(Ax-Bx, 2)+pow(Ay-By, 2));
                if (distance > 10){
                    printf("Distance too big\n");
                    break;
                }


                double f1 = (By - Ay)*Rx + (Ax-Bx)*Ry + (Bx*Ay-Ax*By);
                double f2 = (By - Ay)*Sx + (Ax-Bx)*Sy + (Bx*Ay-Ax*By);
                double f3 = (By - Ay)*Tx + (Ax-Bx)*Ty + (Bx*Ay-Ax*By);
                double f4 = (By - Ay)*Ux + (Ax-Bx)*Uy + (Bx*Ay-Ax*By);

                if (f1 < 0){
                    negativeFs++;
                } else if (f1 != 0){
                    positiveFs++;
                }
                if (f2 < 0){
                    negativeFs++;
                } else if (f2 != 0){
                    positiveFs++;
                }
                if (f3 < 0){
                    negativeFs++;
                } else if (f3 != 0){
                    positiveFs++;
                }
                if (f4 < 0){
                    negativeFs++;
                } else if (f4 != 0){
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
                    if (!overOccupiedSquare){
                        printf(" --- passed\n");
                    }


                    if (overOccupiedSquare){
                        // this is not a collision free path
                        printf(" --- broke\n");
                        break;
                    }
                }
                
            }
            
            if (!overOccupiedSquare){ 
                        // a collision free path can be found so draw it

                        LCDLine(Ay, Ax, By, Bx, BLUE); // Draw it on screen

                        int distance = sqrt(pow(Ax-Bx, 2)+pow(Ay-By, 2));

                        printf("Distance From (%i, %i) -> (%i, %i): %i\n", Ax, Ay, Bx, By, distance);
                        
                        // vector<vector<int> > path2d;
                        // vector<int> path1da;
                        // vector<int> path1db;
                        // path1da.push_back(Ax);
                        // path1da.push_back(Ay);
                        // path1db.push_back(Bx);
                        // path1db.push_back(By);

                        // path2d.push_back(path1da);
                        // path2d.push_back(path1db);

                        // paths->path.push_back(path2d);

            pathCount++;
            } else {
                printf("Did not print\n");
            }
            
        }
    }
}

void driveToPoints()
{
    // TODO
}

void printfImage(BYTE img)
{
    for (int i = 0; i < IMAGE_SIZE; i++)
    {
        for (int j = 0; j < IMAGE_SIZE; j++)
        {
            // Both %d and %X works
            printf("%d", *image);
            
            
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
            collisionFreePaths();
            break;
        case KEY3:
            printf("\nExperiment 4\n---\n");
            // driveToPoints();
            break;
        case KEY4:
            endSim = 1;
            break;
        }
    } while (!endSim);
}