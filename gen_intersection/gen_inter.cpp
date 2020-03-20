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
#define _USE_MATH_DEFINES

#define STEP_SIZE 0.05  // 0.05 lane width (around 10cm)
#define MIN_DIFF 1    // 1 lane width

#define LEN_OFFSET 0    // SUMO network turning radias (# of lane width)

using namespace std;


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
    
    
    // Get all pair of combination
    #pragma omp parallel for collapse(4) schedule(dynamic)
    for (int lane_1 = 0; lane_1 < lane_num_per_dir; lane_1++){
        for (int dir_1 = 0; dir_1 < dir_num; dir_1++){
            for (int lane_2 = 0; lane_2 < lane_num_per_dir*4; lane_2++){
                for (int dir_2 = 0; dir_2 < dir_num; dir_2++){
                    if (lane_1 == lane_2 && dir_1 == dir_2){
                        // Skip same direction, same lane cars
                        continue;
                    }
                    
                    // Get the name of current iteration
                    string name = to_string(lane_1)+directions[dir_1]+to_string(lane_2)+directions[dir_2];
                    string result = name;
                    
                    // S to S
                    if (directions[dir_1] == 'S' && directions[dir_2] == 'S'){
                        if (lane_2<lane_num_per_dir){
                            continue;
                        }
                        else if (lane_2 >= lane_num_per_dir*2 && lane_2 < lane_num_per_dir*3){
                            continue;
                        }
                        else{
                            
                            Line* line_1 = new Line(lane_num_per_dir+lane_1+1+LEN_OFFSET, 0, 0);
                            Line* line_2 = new Line();
                            
                            // Setup lane 2
                            if (lane_2 < lane_num_per_dir*2){
                                line_2->setType(2);
                                line_2->setX(lane_num_per_dir*2+1+2*LEN_OFFSET);
                                line_2->setY(lane_num_per_dir+(lane_2-lane_num_per_dir)+1+LEN_OFFSET);
                            }
                            else if(lane_2 >= lane_num_per_dir*3){
                                line_2->setType(3);
                                line_2->setX(0);
                                line_2->setY(lane_num_per_dir-(lane_2-lane_num_per_dir*3)+LEN_OFFSET);
                            }
                            
                            // Determine lane steps
                            double max_step = lane_num_per_dir*2+1+2*LEN_OFFSET;
                            
                            //    ((Xm, Ym), (Xd, Yd), (Pi, Pj))
                            double Xm = max_step;
                            double Ym = max_step;
                            double Xd = 0;
                            double Yd = 0;
                                    
                            for (double idx_1 = 0; idx_1 < max_step; idx_1+=STEP_SIZE){
                                for (double idx_2 = 0; idx_2 < max_step; idx_2+=STEP_SIZE){
                                    Point *pt1 = line_1->getPointFromStep(idx_1);
                                    Point *pt2 = line_2->getPointFromStep(idx_2);
                                    
                                    double distance = pt1->Distance(*pt2);
                                    if (distance<MIN_DIFF){
                                        Xm = max(0.0,min(Xm, idx_1-STEP_SIZE));
                                        Ym = max(0.0,min(Ym, idx_2-STEP_SIZE));
                                        
                                        Xd = min(max_step,max(Xd, idx_1+STEP_SIZE));
                                        Yd = min(max_step,max(Yd, idx_2+STEP_SIZE));
                                    }
                                    
                                    delete pt1;
                                    delete pt2;
                                }
                            }
                            
                            if (Xm == max_step && Ym == max_step){
                                // No collision
                                continue;
                            }
                            
                            result += " ";
                            result += to_string(Xm);
                            result += " ";
                            result += to_string(Ym);
                            
                            result += " ";
                            result += to_string(Xd);
                            result += " ";
                            result += to_string(Yd);
                            
                            // Add (Pi, Pj)
                            result += " ";
                            result += to_string(max_step);
                            result += " ";
                            result += to_string(max_step);
                            
                            
                            delete line_1;
                            delete line_2;
                        }
                    }
                    
                    // L/R to L/R
                    else if (directions[dir_1] != 'S' && directions[dir_2] != 'S'){

                        Circle* cir_1 = new Circle();
                        Circle* cir_2 = new Circle();
                        
                        // Setup circles
                        if (directions[dir_1] == 'L'){
                            Point center(0,0);
                            cir_1->setCenter(center);
                            cir_1->setR(lane_1+lane_num_per_dir+1+LEN_OFFSET);
                            cir_1->setTurn(1);
                            cir_1->setType(0);
                        }
                        else if (directions[dir_1] == 'R'){
                            Point center(lane_num_per_dir*2+1+2*LEN_OFFSET,0);
                            cir_1->setCenter(center);
                            cir_1->setR((lane_num_per_dir*2+1+2*LEN_OFFSET)-(lane_1+lane_num_per_dir+1+LEN_OFFSET));
                            cir_1->setTurn(0);
                            cir_1->setType(1);
                        }
                        
                        if (directions[dir_2] == 'L'){
                            cir_2->setR((lane_2%lane_num_per_dir)+lane_num_per_dir+1+LEN_OFFSET);
                            cir_2->setTurn(1);
                            
                            int dir = lane_2/lane_num_per_dir;
                            if (dir == 0){
                                Point center(0,0);
                                cir_2->setCenter(center);
                                cir_2->setType(0);
                            }
                            else if (dir == 1){
                                Point center(lane_num_per_dir*2+1+2*LEN_OFFSET,0);
                                cir_2->setCenter(center);
                                cir_2->setType(1);
                            }
                            else if (dir == 2){
                                Point center(lane_num_per_dir*2+1+2*LEN_OFFSET,lane_num_per_dir*2+1+2*LEN_OFFSET);
                                cir_2->setCenter(center);
                                cir_2->setType(2);
                            }
                            else if (dir == 3){
                                Point center(0,lane_num_per_dir*2+1+2*LEN_OFFSET);
                                cir_2->setCenter(center);
                                cir_2->setType(3);
                            }
                        }
                        else if (directions[dir_2] == 'R'){
                            cir_2->setR((lane_num_per_dir*2+1+2*LEN_OFFSET)-(lane_2%lane_num_per_dir)-(lane_num_per_dir+1+LEN_OFFSET));
                            cir_2->setTurn(0);
                            
                            int dir = lane_2/lane_num_per_dir;
                            if (dir == 0){
                                Point center(lane_num_per_dir*2+1+2*LEN_OFFSET,0);
                                cir_2->setCenter(center);
                                cir_2->setType(1);
                            }
                            else if (dir == 1){
                                Point center(lane_num_per_dir*2+1+2*LEN_OFFSET,lane_num_per_dir*2+1+2*LEN_OFFSET);
                                cir_2->setCenter(center);
                                cir_2->setType(2);
                            }
                            else if (dir == 2){
                                Point center(0,lane_num_per_dir*2+1+2*LEN_OFFSET);
                                cir_2->setCenter(center);
                                cir_2->setType(3);
                            }
                            else if (dir == 3){
                                Point center(0,0);
                                cir_2->setCenter(center);
                                cir_2->setType(0);
                            }
                        }
                        
                        
                        // Determine lane steps
                        double max_step_1 = cir_1->getMaxStep();
                        double max_step_2 = cir_2->getMaxStep();
                        
                        //    ((Xm, Ym), (Xd, Yd), (Pi, Pj))
                        double Xm = max_step_1;
                        double Ym = max_step_2;
                        double Xd = 0;
                        double Yd = 0;
                        
                        for (double idx_1 = 0; idx_1 < max_step_1; idx_1+=STEP_SIZE){
                            for (double idx_2 = 0; idx_2 < max_step_2; idx_2+=STEP_SIZE){
                                Point *pt1 = cir_1->getPointFromStep(idx_1);
                                Point *pt2 = cir_2->getPointFromStep(idx_2);
                                
                                double distance = pt1->Distance(*pt2);
                                if (distance<MIN_DIFF){
                                    Xm = max(0.0,min(Xm, idx_1-STEP_SIZE));
                                    Ym = max(0.0,min(Ym, idx_2-STEP_SIZE));
                                    
                                    Xd = min(max_step_1,max(Xd, idx_1+STEP_SIZE));
                                    Yd = min(max_step_2,max(Yd, idx_2+STEP_SIZE));
                                }
                                
                                delete pt1;
                                delete pt2;
                            }
                        }
                        
                        if (Xm == max_step_1 && Ym == max_step_2){
                            // No collision
                            continue;
                        }
                        
                        result += " ";
                        result += to_string(Xm);
                        result += " ";
                        result += to_string(Ym);
                        
                        result += " ";
                        result += to_string(Xd);
                        result += " ";
                        result += to_string(Yd);
                        
                        // Add (Pi, Pj)
                        result += " ";
                        result += to_string(max_step_1);
                        result += " ";
                        result += to_string(max_step_2);
                        
                        
                        delete cir_1;
                        delete cir_2;
                    }
                    
                    // S to L/R
                    else if (directions[dir_1] == 'S' && directions[dir_2] != 'S'){

                        Line* line_1 = new Line(lane_num_per_dir+lane_1+1+LEN_OFFSET, 0, 0);
                        Circle* cir_2 = new Circle();
                        
                        // Setup circles
                        
                        if (directions[dir_2] == 'L'){
                            cir_2->setR((lane_2%lane_num_per_dir)+lane_num_per_dir+1+LEN_OFFSET);
                            cir_2->setTurn(1);
                            
                            int dir = lane_2/lane_num_per_dir;
                            if (dir == 0){
                                Point center(0,0);
                                cir_2->setCenter(center);
                                cir_2->setType(0);
                            }
                            else if (dir == 1){
                                Point center(lane_num_per_dir*2+1+2*LEN_OFFSET,0);
                                cir_2->setCenter(center);
                                cir_2->setType(1);
                            }
                            else if (dir == 2){
                                Point center(lane_num_per_dir*2+1+2*LEN_OFFSET,lane_num_per_dir*2+1+2*LEN_OFFSET);
                                cir_2->setCenter(center);
                                cir_2->setType(2);
                            }
                            else if (dir == 3){
                                Point center(0,lane_num_per_dir*2+1+2*LEN_OFFSET);
                                cir_2->setCenter(center);
                                cir_2->setType(3);
                            }
                        }
                        else if (directions[dir_2] == 'R'){
                            cir_2->setR(lane_num_per_dir*2+1+2*LEN_OFFSET-(lane_2%lane_num_per_dir)-(lane_num_per_dir+1+LEN_OFFSET));
                            cir_2->setTurn(0);
                            
                            int dir = lane_2/lane_num_per_dir;
                            if (dir == 0){
                                Point center(lane_num_per_dir*2+1+2*LEN_OFFSET,0);
                                cir_2->setCenter(center);
                                cir_2->setType(1);
                            }
                            else if (dir == 1){
                                Point center(lane_num_per_dir*2+1+2*LEN_OFFSET,lane_num_per_dir*2+1+2*LEN_OFFSET);
                                cir_2->setCenter(center);
                                cir_2->setType(2);
                            }
                            else if (dir == 2){
                                Point center(0,lane_num_per_dir*2+1+2*LEN_OFFSET);
                                cir_2->setCenter(center);
                                cir_2->setType(3);
                            }
                            else if (dir == 3){
                                Point center(0,0);
                                cir_2->setCenter(center);
                                cir_2->setType(0);
                            }
                        }
                        
                        
                        // Determine lane steps
                        double max_step_1 = lane_num_per_dir*2+1+2*LEN_OFFSET;
                        double max_step_2 = cir_2->getMaxStep();
                        
                        //    ((Xm, Ym), (Xd, Yd), (Pi, Pj))
                        double Xm = max_step_1;
                        double Ym = max_step_2;
                        double Xd = 0;
                        double Yd = 0;
                        
                        for (double idx_1 = 0; idx_1 < max_step_1; idx_1+=STEP_SIZE){
                            for (double idx_2 = 0; idx_2 < max_step_2; idx_2+=STEP_SIZE){
                                Point *pt1 = line_1->getPointFromStep(idx_1);
                                Point *pt2 = cir_2->getPointFromStep(idx_2);
                                
                                double distance = pt1->Distance(*pt2);
                                if (distance<MIN_DIFF){
                                    Xm = max(0.0,min(Xm, idx_1-STEP_SIZE));
                                    Ym = max(0.0,min(Ym, idx_2-STEP_SIZE));
                                    
                                    Xd = min(max_step_1,max(Xd, idx_1+STEP_SIZE));
                                    Yd = min(max_step_2,max(Yd, idx_2+STEP_SIZE));
                                }
                                
                                delete pt1;
                                delete pt2;
                            }
                        }
                        
                        if (Xm == max_step_1 && Ym == max_step_2){
                            // No collision
                            continue;
                        }
                        
                        result += " ";
                        result += to_string(Xm);
                        result += " ";
                        result += to_string(Ym);
                        
                        result += " ";
                        result += to_string(Xd);
                        result += " ";
                        result += to_string(Yd);
                        
                        // Add (Pi, Pj)
                        result += " ";
                        result += to_string(max_step_1);
                        result += " ";
                        result += to_string(max_step_2);
                        
                        delete line_1;
                        delete cir_2;
                    }
                    
                    // L/R to S
                    else if (directions[dir_1] != 'S' && directions[dir_2] == 'S'){
                        
                        Circle* cir_1 = new Circle();
                        Line* line_2 = new Line();
                        
                        // Setup circles
                        if (directions[dir_1] == 'L'){
                            Point center(0,0);
                            cir_1->setCenter(center);
                            cir_1->setR(lane_1+lane_num_per_dir+1+LEN_OFFSET);
                            cir_1->setTurn(1);
                            cir_1->setType(0);
                        }
                        else if (directions[dir_1] == 'R'){
                            Point center(lane_num_per_dir*2+1+2*LEN_OFFSET,0);
                            cir_1->setCenter(center);
                            cir_1->setR((lane_num_per_dir*2+1+2*LEN_OFFSET)-lane_1-(lane_num_per_dir+1+LEN_OFFSET));
                            cir_1->setTurn(0);
                            cir_1->setType(1);
                        }
                        
                        // Setup lane 2
                        if (lane_2 < lane_num_per_dir){
                            line_2->setType(0);
                            line_2->setX(lane_num_per_dir+lane_2+1+LEN_OFFSET);
                            line_2->setY(0);
                        }
                        else if (lane_2 < lane_num_per_dir*2){
                            line_2->setType(2);
                            line_2->setX(lane_num_per_dir*2+1+2*LEN_OFFSET);
                            line_2->setY(lane_num_per_dir+(lane_2-lane_num_per_dir)+1+LEN_OFFSET);
                        }
                        else if (lane_2 < lane_num_per_dir*3){
                            line_2->setType(1);
                            line_2->setX((lane_num_per_dir+LEN_OFFSET)-(lane_2-lane_num_per_dir*2));
                            line_2->setY(lane_num_per_dir*2+1+2*LEN_OFFSET);
                        }
                        else if(lane_2 < lane_num_per_dir*4){
                            line_2->setType(3);
                            line_2->setX(0);
                            line_2->setY((lane_num_per_dir+LEN_OFFSET)-(lane_2-lane_num_per_dir*3));
                        }
                        
                        
                        // Determine lane steps
                        double max_step_1 = cir_1->getMaxStep();
                        double max_step_2 = lane_num_per_dir*2+1+2*LEN_OFFSET;
                        
                        //    ((Xm, Ym), (Xd, Yd), (Pi, Pj))
                        double Xm = max_step_1;
                        double Ym = max_step_2;
                        double Xd = 0;
                        double Yd = 0;
                        
                        for (double idx_1 = 0; idx_1 < max_step_1; idx_1+=STEP_SIZE){
                            for (double idx_2 = 0; idx_2 < max_step_2; idx_2+=STEP_SIZE){
                                Point *pt1 = cir_1->getPointFromStep(idx_1);
                                Point *pt2 = line_2->getPointFromStep(idx_2);
                                
                                double distance = pt1->Distance(*pt2);
                                if (distance<MIN_DIFF){
                                    
                                    Xm = max(0.0,min(Xm, idx_1-STEP_SIZE));
                                    Ym = max(0.0,min(Ym, idx_2-STEP_SIZE));
                                    
                                    Xd = min(max_step_1,max(Xd, idx_1+STEP_SIZE));
                                    Yd = min(max_step_2,max(Yd, idx_2+STEP_SIZE));
                                }
                                
                                delete pt1;
                                delete pt2;
                            }
                        }
                        
                        if (Xm == max_step_1 && Ym == max_step_2){
                            // No collision
                            continue;
                        }
                        
                        result += " ";
                        result += to_string(Xm);
                        result += " ";
                        result += to_string(Ym);
                        
                        result += " ";
                        result += to_string(Xd);
                        result += " ";
                        result += to_string(Yd);
                        
                        // Add (Pi, Pj)
                        result += " ";
                        result += to_string(max_step_1);
                        result += " ";
                        result += to_string(max_step_2);
                        
                        delete cir_1;
                        delete line_2;
                    }
                    
                    #pragma omp critical
                    {
                        f_out << result << endl;
                        f_out.flush();
                    }
                    
                }
            }
        }
    }
    
    
    
    return 0;
}