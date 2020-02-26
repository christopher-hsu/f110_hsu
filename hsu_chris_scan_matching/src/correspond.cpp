#include "scan_matching_skeleton/correspond.h"
#include "cmath"
#include "ros/ros.h"

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){

      c.clear();
      int last_best = -1;
      const int n = trans_points.size();
      const int m = old_points.size();
      float min_dist = 100000.00;
      int min_index = 0;
      int second_min_index = 0;

      //Do for each point
      for(int i = 0; i<n; ++i){
        for(int j = 0; j<m; ++j){
          float dist = old_points[i].distToPoint2(&trans_points[j]);
          if(dist<min_dist){
            min_dist = dist;
            min_index = j;
            second_min_index = j-1;
          }
        }
        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
      }


}

void getCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){

  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // use helper functions and structs in transform.h and correspond.h
  // input : old_points : vector of struct points containing the old points (points of the previous frame)
  // input : trans_points : vector of struct points containing the new points transformed to the previous frame using the current estimated transform
  // input : points : vector of struct points containing the new points
  // input : jump_table : jump table computed using the helper functions from the transformed and old points
  // input : c: vector of struct correspondences . This is a refernece which needs to be updated in place and return the new correspondences to calculate the transforms.
  // output : c; update the correspondence vector in place which is provided as a reference. you need to find the index of the best and the second best point. 
  //Initializecorrespondences
  c.clear();
  int last_best = -1;
  const int n = trans_points.size();
  const int m = old_points.size();

  //Do for each point
  for(int i = 0; i<n; ++i){
    Point p_i_w = trans_points[i];
    // Initialization
    // Current best match and dist
    int best = -1;
    int second_best = -1;

    int best_up = -1;
    int best_down = -1;
    double best_up_dist = 9999999.9;
    double best_down_dist = 9999999.9;

    // Approx index in old scan of trans point p_i_w
    p_i_w.wrapTheta();
    int start_idx = (p_i_w.theta + M_PI) * (m/(2*M_PI)); //check if m is correct
    // Starting search index
    int we_start_at = (last_best != -1) ? (last_best + 1) : start_idx;
    int up = we_start_at+1, down = we_start_at;
    // Distance of last point examined
    double last_dist_up = 9999999.9, last_dist_down = 9999999.9;
    // Stopping conditions of search
    bool up_stopped = false, down_stopped = false;

    while (!(up_stopped && down_stopped)) {
      // should we explore up or down
      bool now_up = !up_stopped & (last_dist_up <= last_dist_down);
      if (now_up) {
        if (up >= m-1) {
          if (best_up == -1) {
            best_up = m-1;
          }
          up_stopped = true;
          continue;
        }
        last_dist_up = (p_i_w.distToPoint2(&old_points[up]));
        if (last_dist_up < best_up_dist) {
          best_up = up;
          best_up_dist = last_dist_up;
        }
        if (up > start_idx) {
          double d_phi = old_points[up].theta - p_i_w.theta;
          double min_dist_up = sin(d_phi) * p_i_w.r;
          if (pow(min_dist_up,2) > best_up_dist){
            up_stopped = true; 
            continue;
          }
          up = (old_points[up].r < p_i_w.r) ? jump_table[up][UP_BIG] : jump_table[up][UP_SMALL];
        }
        else {
          up++;
        }
      }
      if (!now_up) {
        if (down <= 0) {
          if (best_down == -1) {
            best_down = 0;
          }
          down_stopped = true;
          continue;
        }
        last_dist_down = (p_i_w.distToPoint2(&old_points[down]));
        if (last_dist_down < best_down_dist) {
          best_down = down;
          best_down_dist = last_dist_down;
        }
        if (down < start_idx) {
          double d_phi = old_points[down].theta - p_i_w.theta;
          double min_dist_down = sin(d_phi) * p_i_w.r;
          if (pow(min_dist_down,2) > best_down_dist){
            down_stopped = true; 
            continue;
          }
          down = (old_points[down].r < p_i_w.r) ? jump_table[down][DOWN_BIG] : jump_table[down][DOWN_SMALL];
        }
        else {
          down--;
        }
      }
    }
    // Determine best and second best
    if (best_down_dist < best_up_dist) {
      best = best_down;
      second_best = best_down + 1;
    }
    else {
      best = best_up;
      second_best = best_up - 1;
    }
    last_best = best;

    c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[best], &old_points[second_best]));
  }
}


void computeJump(vector< vector<int> >& table, vector<Point>& points){
  table.clear();
  int n = points.size();
  for(int i = 0; i<n; ++i){
    vector<int> v = {n,n,-1,-1};
    for(int j = i+1; j<n; ++j){
      if(points[j].r<points[i].r){
        v[UP_SMALL] = j;
        break;
      }
    }
    for(int j = i+1; j<n; ++j){
      if(points[j].r>points[i].r){
        v[UP_BIG] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r<points[i].r){
        v[DOWN_SMALL] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r>points[i].r){
        v[DOWN_BIG] = j;
        break;
      }
    }
    table.push_back(v);
  }
}
