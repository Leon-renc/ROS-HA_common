#include "node2d.h"

using namespace HybridAStar;

// possible directions
const int Node2D::dir = 8;
// possible movements
const int Node2D::dx[] = { -1, -1, 0, 1, 1, 1, 0, -1 };
const int Node2D::dy[] = { 0, 1, 1, 1, 0, -1, -1, -1 };

const int  Node2D::radius = 5;
const int Node2D::dir_x =2*radius+1; 
const int Node2D::dir_y =2*radius+1; 

//###################################################
//                                         IS ON GRID
//###################################################
bool Node2D::isOnGrid(const int width, const int height) const {
 //--Mapping debug--
  //return  x >= 0 && x < width && y >= 0 && y < height;
  bool flag_1 = (x - map_origin_x)/map_resolution >= 0 && (x - map_origin_x)/map_resolution <= width;
  bool flag_2 = (y - map_origin_y)/map_resolution >= 0 && (y - map_origin_y)/map_resolution <= height;
  return flag_1 && flag_2;
}

//###################################################
//                                   CREATE SUCCESSOR
//###################################################
Node2D* Node2D::createSuccessor(const int i) {
  int xSucc = x + Node2D::dx[i];
  int ySucc = y + Node2D::dy[i];
  return new Node2D(xSucc, ySucc, g, 0, this);
}

Node2D* Node2D::createSuccessor(const int i, const int j) {
  int xSucc = x - Node2D::radius + i;
  int ySucc = y - Node2D::radius + j;
  return new Node2D(xSucc, ySucc, g, 0, this);
}

bool Node2D::isReachable(const Node2D* nPred, const Node2D* goal)
{
  //std::cout << "nSucc: (" << x <<"   " <<  y <<")" << "nPred: (" << nPred->getX() << "   " <<  nPred->getY() <<" )" << "goal: (" << goal->getX() << "   " <<  goal->getY() <<" )" << std::endl;
  if (nPred->getX() < goal->getX() && x <= goal->getX() && x >= nPred->getX())
  {
    return true;
  }
  if(nPred->getX() >= goal->getX() &&x >= goal->getX() && x <= nPred->getX())
  {
    return true;
  }

  if (nPred->getY() < goal->getY() && y <= goal->getY() && y >= nPred->getY())
  {
    return true;
  }
  if (nPred->getY() >= goal->getY() && y >= goal->getY() && y<= nPred->getY())
  {
   return true;
  }
  return false;
}

 bool Node2D::willCollision(const Node2D* currentCenter, std::vector<Node2D>* obsInfo,  const Node2D* goal)
 {
   if ( !obsInfo->size() )
   {
     return false;
   }
   else 
   {
     if (meetDistanceAndScopeCondition( currentCenter, obsInfo, goal))
     {
       return true;
     }
     return false;
   }
 }

 bool Node2D::meetDistanceAndScopeCondition(const Node2D* currentCenter, std::vector<Node2D>* obsInfo, const Node2D* goal)
 {
   int currentCenter_x = currentCenter->getX();
   int currentCenter_y = currentCenter->getY();
   int goal_x = goal ->getX();
   int goal_y = goal->getY();
    for (auto iter = obsInfo->begin(); iter != obsInfo->end(); iter++)
    {
      int obs_x = iter->getX();
      int obs_y = iter->getY();
      float distCB = sqrt((currentCenter_x - obs_x)*(currentCenter_x - obs_x) + (currentCenter_y - obs_y)*(currentCenter_y - obs_y));
      float distCn = sqrt((currentCenter_x - x) * (currentCenter_x - x) + (currentCenter_y - y) * (currentCenter_y - y));
      float distCT = sqrt((currentCenter_x - goal_x) * (currentCenter_x - goal_x) + (currentCenter_y - goal_y) * (currentCenter_y - goal_y));
      if ( distCn > distCB && distCn < distCT)
      {
        float k1 = ((obs_y + 0.5) - currentCenter_y)/(obs_x - 0.5 - currentCenter_x);
        float k2 = ((obs_y + 0.5) - currentCenter_y)/(obs_x + 0.5 - currentCenter_x);
        float k3 = ((obs_y - 0.5) - currentCenter_y)/(obs_x + 0.5 - currentCenter_x);
        float k4 = ((obs_y - 0.5) - currentCenter_y)/(obs_x - 0.5 - currentCenter_x);
        float k = (goal_y - currentCenter_y)/ (goal_x - currentCenter->getX());
        if ( k > std::min({k1, k2, k3, k4}) && k < std::max ({k1, k2, k3, k4}) )
        {
          std::cout << "there is a obs (" << obs_x <<", "<<obs_y << ")" << "so, current center (" << currentCenter_x << ", " << currentCenter_y  << ") can not achieve the goal" << std::endl; 
          return true;
        }
      }
    }
     return false;
 }

 void Node2D::calculateAndSetG( std::vector<Node2D>  &reachableGrid)
{
  float totalDistCost = 0.f;
  if ( !reachableGrid.size() )
  {
    std::cout << "Disable to calculate G , due to invaild reachableGrid" << std::endl ;
  }
  for (int i = 0; i < reachableGrid.size(); i++)
  {
    totalDistCost += sqrt((x - reachableGrid[i].getX()) * (x - reachableGrid[i].getX()) + (y - reachableGrid[i].getY()) * (y - reachableGrid[i].getY()));
  }
    for (int i = 0; i < reachableGrid.size(); i++)
  {
    reachableGrid[i].setW(( totalDistCost - sqrt((x - reachableGrid[i].getX()) * (x - reachableGrid[i].getX()) + 
    (y - reachableGrid[i].getY()) * (y - reachableGrid[i].getY()))) / totalDistCost);
    //std::cout << "node :" << reachableGrid[i].getX() <<  "," << reachableGrid[i].getY() << " G is" << reachableGrid[i].getG() << " W is" << reachableGrid[i].getW() << std::endl;
    reachableGrid[i].setG(reachableGrid[i].getW()*reachableGrid[i].getG() );
  }
}



//###################################################
//                                 2D NODE COMPARISON
//###################################################
bool Node2D::operator == (const Node2D& rhs) const {
  return x == rhs.x && y == rhs.y;
}
