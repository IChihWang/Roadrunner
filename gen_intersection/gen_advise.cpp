/*
     5 4 _ _ _
    |_|_|_|_|_| 3
    |_|_|_|_|_| 2
    |_|_|_|_|_|
   6|_|_|_|_|_|
   7|_|_|_|_|_|
           0 1
           ^ ^ lane_1

(carX, carY): (Xm, Ym), (Xd, Yd), (Px, Py)

*/

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <cmath>
#include <omp.h>
#include <algorithm>
#include <vector>
#define _USE_MATH_DEFINES

#define STEP_SIZE 0.05  // 0.05 lane width (around 10cm)
#define MIN_DIFF 1.5    // 1 lane width
#define RESOLVE 2    // 2 blocks per lane width

using namespace std;

/* Blocks */
class Block{
public:
    int x;
    int y;
    double distance;

    bool operator==(const Block& b1){return (b1.x==this->x && b1.y==this->y);}
};

/* Points */
class Point{
private:
    double x;
    double y;

public:
    Point(){}
    Point(double in_x, double in_y);
    double Distance(Point pt);
    double getX(){return x;}
    double getY(){return y;}
    void setXY(double x, double y){this->x = x;this->y = y;}
};

Point::Point(double in_x, double in_y){
    this->x = in_x;
    this->y = in_y;
}
double Point::Distance(Point pt){
    return sqrt(pow(pt.x-this->x, 2) + pow(pt.y-this->y, 2));
}



/* Circle class */
class Circle{
private:
    Point center;
    double r;
    short turn = -1;     // 0: clockwise (Right), 1: anticlockwise (Left)
    short type = -1;       // 0: ↙  1: ↘  2:  ↗  3: ↖

public:
    Circle(){}
    Circle(Point c, double r);

    // Only handle positive values
    Point* getPointFromX(double x, bool is_up);
    void setCenter(Point center){this->center = center;}
    void setR(double r){this->r = r;}
    double getMaxStep();
    Point* getPointFromStep(double step);
    void setType(short type){this->type = type;}
    void setTurn(short turn){this->turn = turn;}

};

Circle::Circle(Point c, double r){
    this->center = c;
    this->r = r;
}

Point* Circle::getPointFromX(double x, bool is_up){
    double y;
    if (abs(this->center.getX() - x) > r){
        return NULL;
    }
    else{
        double tmp_ans = sqrt( this->r - pow(x-this->center.getX(), 2) );
        if (is_up){
            y = this->center.getY()+tmp_ans;
        }
        else{
            y = this->center.getY()-tmp_ans;
        }
    }
    return (new Point(x,y));
}

double Circle::getMaxStep(){
    return 2*M_PI*this->r/4;
}

Point* Circle::getPointFromStep(double step){
    double angle = step/this->r;

    // Set turn & angle
    if (this->turn == 0)
        angle = -angle;
    else if (this->turn == 1)
        angle = angle;
    else{
        cout << "Circle is not set properly angle." << endl;
        exit(0);
    }

    // Set beginning point
    if (this->type == 0){
        if (this->turn == 0)
            angle += M_PI/2;
        else if (this->turn == 1)
            angle += 0;
    }
    else if (this->type == 1){
        if (this->turn == 0)
            angle += M_PI;
        else if (this->turn == 1)
            angle += M_PI/2;
    }
    else if (this->type == 2){
        if (this->turn == 0)
            angle += 3*M_PI/2;
        else if (this->turn == 1)
            angle += M_PI;
    }
    else if (this->type == 3){
        if (this->turn == 0)
            angle += 0;
        else if (this->turn == 1)
            angle += 3*M_PI/2;
    }
    else{
        cout << "Circle is not set properly turn." << endl;
        exit(0);
    }

    // Get Point
    Point *pt = new Point();
    double x = cos(angle)*this->r;
    double y = sin(angle)*this->r;
    pt->setXY(x+this->center.getX(),y+this->center.getY());

    return pt;
}



/* Line class */
class Line{
private:
    double x;       // Starting point x
    double y;       // Starting point y
    short type;      // 0: ↑  1: ↓  2: ←   3: →

public:
    Line(){}
    Line(double x, double y, short type){this->x = x;this->y = y;this->type=type;}

    Point* getPointFromStep(double step);
    void setType(short type){this->type=type;}
    void setX(double x){this->x = x;}
    void setY(double y){this->y = y;}
};

Point* Line::getPointFromStep(double step){
    Point *pt = new Point();
    if (type == 0){
        pt->setXY(this->x, step);
    }
    else if (type == 1){
        pt->setXY(this->x, this->y-step);
    }
    else if (type == 2){
        pt->setXY(this->x-step, this->y);
    }
    else if (type == 3){
        pt->setXY(step, this->y);
    }

    return pt;
}



int main(int argc, char* argv[]){
    if (argc < 3){
        cout << "Usage: " << argv[0] << " <lane num per direction> <output name>" << endl;
        return -1;
    }

    ofstream f_out(argv[2]);

    int lane_num_per_dir = atoi(argv[1]);
    char directions[] = {'L', 'S', 'R'};
    int dir_num = sizeof(directions)/sizeof(directions[0]);


    #pragma omp parallel for collapse(2) schedule(dynamic)
    for (int lane_i = 0; lane_i < lane_num_per_dir; lane_i++){
        for (int dir_i = 0; dir_i < dir_num; dir_i++){
            vector<Block> block_vec;
            if (directions[dir_i] == 'S'){
                Line* line = new Line(lane_num_per_dir+lane_i+0.6, 0, 0);

                double max_step = lane_num_per_dir*2+1;
                for (double idx_1 = 0; idx_1 < max_step; idx_1+=STEP_SIZE){
                    Point *pt1 = line->getPointFromStep(idx_1);
                    Block block;
                    block.x = int(pt1->getX()*RESOLVE);
                    block.y = int(pt1->getY()*RESOLVE);
                    block.distance = idx_1;

                    if(find(block_vec.begin(), block_vec.end(), block) == block_vec.end()){
                        block_vec.push_back(block);
                    }
                }

                Line* line_2 = new Line(lane_num_per_dir+lane_i+1.4, 0, 0);

                double max_step_2 = lane_num_per_dir*2+1;
                for (double idx_1 = 0; idx_1 < max_step_2; idx_1+=STEP_SIZE){
                    Point *pt1 = line_2->getPointFromStep(idx_1);
                    Block block;
                    block.x = int(pt1->getX()*RESOLVE);
                    block.y = int(pt1->getY()*RESOLVE);
                    block.distance = idx_1;

                    if(find(block_vec.begin(), block_vec.end(), block) == block_vec.end()){
                        block_vec.push_back(block);
                    }
                }
            }
            else{
                Circle* cir_1 = new Circle();

                // Setup circles
                if (directions[dir_i] == 'L'){
                    Point center(0,0);
                    cir_1->setCenter(center);
                    cir_1->setR(lane_i+lane_num_per_dir+0.6);
                    cir_1->setTurn(1);
                    cir_1->setType(0);
                }
                else if (directions[dir_i] == 'R'){
                    Point center(lane_num_per_dir*2+1,0);
                    cir_1->setCenter(center);
                    cir_1->setR(lane_num_per_dir*2+0.6-lane_i-(lane_num_per_dir+1));
                    cir_1->setTurn(0);
                    cir_1->setType(1);
                }

                double max_step = cir_1->getMaxStep();
                for (double idx_1 = 0; idx_1 < max_step; idx_1+=STEP_SIZE){
                    Point *pt1 = cir_1->getPointFromStep(idx_1);
                    Block block;
                    block.x = int(pt1->getX()*RESOLVE);
                    block.y = int(pt1->getY()*RESOLVE);
                    block.distance = idx_1;
                    if(find(block_vec.begin(), block_vec.end(), block) == block_vec.end()){
                        block_vec.push_back(block);
                    }
                }


                Circle* cir_2 = new Circle();

                // Setup circles
                if (directions[dir_i] == 'L'){
                    Point center(0,0);
                    cir_2->setCenter(center);
                    cir_2->setR(lane_i+lane_num_per_dir+1.4);
                    cir_2->setTurn(1);
                    cir_2->setType(0);
                }
                else if (directions[dir_i] == 'R'){
                    Point center(lane_num_per_dir*2+1,0);
                    cir_2->setCenter(center);
                    cir_2->setR(lane_num_per_dir*2+1.4-lane_i-(lane_num_per_dir+1));
                    cir_2->setTurn(0);
                    cir_2->setType(1);
                }

                double max_step_2 = cir_2->getMaxStep();
                for (double idx_1 = 0; idx_1 < max_step_2; idx_1+=STEP_SIZE){
                    Point *pt1 = cir_2->getPointFromStep(idx_1);
                    Block block;
                    block.x = int(pt1->getX()*RESOLVE);
                    block.y = int(pt1->getY()*RESOLVE);
                    block.distance = idx_1;
                    if(find(block_vec.begin(), block_vec.end(), block) == block_vec.end()){
                        block_vec.push_back(block);
                    }
                }
            }

             // TODO: write file
            string name = to_string(lane_i)+directions[dir_i];
            string result = name;

            for (int i = 0; i < block_vec.size(); i++){
                result += " ";
                result += to_string(block_vec[i].x);
                result += " ";
                result += to_string(block_vec[i].y);
                result += " ";
                result += to_string(block_vec[i].distance);
            }

            #pragma omp critical
            {
                f_out << result << endl;
                f_out.flush();
            }

        }
    }


    return 0;
}
