#include <stdio.h>
#include <math.h>
#include <stdlib.h>

double * findPlanes(double alphaA, double alphaB, double alphaC);
void setup(){
  Serial.begin(115200);
}

void loop() {
  double *testArray;
  testArray = findPlanes(1.0472, 1.22173, 2.44346);
  Serial.print(testArray[0]); Serial.print(testArray[1]); Serial.println(testArray[2]);
  free(testArray);
}


// This functions finds the determinant of a 3x3 Matrix
double determinantOfMatrix(double mat[3][3])
{
  double ans;
  ans = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2])
        - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
        + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
  return ans;
}


double * findPlanes(double alphaA, double alphaB, double alphaC) {
  double pointA[3] = {-1000, 0, 0};
  double pointB[3] = {0, 0, 0};
  double pointC[3] = {1000, 0, 0};

  double planeA[4] = {0};
  double planeB[4] = {0};
  double planeC[4] = {0};

  double dirA[3] = {cos(alphaA), sin(alphaA), 0};
  double dirB[3] = {0, sin(alphaB), cos(alphaB)};
  double dirC[3] = {cos(alphaC), sin(alphaC), 0};

  double orthA[3] = {0, 0, 1};
  double orthB[3] = {1, 0, 0};
  double orthC[3] = {0, 0, 1};

  double normA[3] = {dirA[1]*orthA[2] - dirA[2]*orthA[1], -(dirA[0]*orthA[2] - dirA[2]*orthA[0]), dirA[0]*orthA[1] - dirA[1]*orthA[0]};
  double normB[3] = {dirB[1]*orthB[2] - dirB[2]*orthB[1], -(dirB[0]*orthB[2] - dirB[2]*orthB[0]), dirB[0]*orthB[1] - dirB[1]*orthB[0]};
  double normC[3] = {dirC[1]*orthC[2] - dirC[2]*orthC[1], -(dirC[0]*orthC[2] - dirC[2]*orthC[0]), dirC[0]*orthC[1] - dirC[1]*orthC[0]};

  planeA[0] = normA[0];
  planeA[1] = normA[1];
  planeA[2] = normA[2];
  planeA[3] = normA[0] * pointA[0] + normA[1] * pointA[1] + normA[2] * pointA[2];

  planeB[0] = normB[0];
  planeB[1] = normB[1];
  planeB[2] = normB[2];
  planeB[3] = normB[0] * pointB[0] + normB[1] * pointB[1] + normB[2] * pointB[2];

  planeC[0] = normC[0];
  planeC[1] = normC[1];
  planeC[2] = normC[2];
  planeC[3] = normC[0] * pointC[0] + normC[1] * pointC[1] + normC[2] * pointC[2];

  double planes[3][4] = {
    { planeA[0], planeA[1], planeA[2], planeA[3]},
    { planeB[0], planeB[1], planeB[2], planeB[3]},
    { planeC[0], planeC[1], planeC[2], planeC[3]}
  };

  // Matrix d using planes as given in cramer's rule
  double d[3][3] = {
    { planes[0][0], planes[0][1], planes[0][2] },
    { planes[1][0], planes[1][1], planes[1][2] },
    { planes[2][0], planes[2][1], planes[2][2] },
  };
  // Matrix d1 using planes as given in cramer's rule
  double d1[3][3] = {
    { planes[0][3], planes[0][1], planes[0][2] },
    { planes[1][3], planes[1][1], planes[1][2] },
    { planes[2][3], planes[2][1], planes[2][2] },
  };
  // Matrix d2 using planes as given in cramer's rule
  double d2[3][3] = {
    { planes[0][0], planes[0][3], planes[0][2] },
    { planes[1][0], planes[1][3], planes[1][2] },
    { planes[2][0], planes[2][3], planes[2][2] },
  };
  // Matrix d3 using planes as given in cramer's rule
  double d3[3][3] = {
    { planes[0][0], planes[0][1], planes[0][3] },
    { planes[1][0], planes[1][1], planes[1][3] },
    { planes[2][0], planes[2][1], planes[2][3] },
  };

  // Calculating Determinant of Matrices d, d1, d2, d3
  double D = determinantOfMatrix(d);
  double D1 = determinantOfMatrix(d1);
  double D2 = determinantOfMatrix(d2);
  double D3 = determinantOfMatrix(d3);

  double x,y,z;
  
  
  if (D != 0) {
     x = D1 / D;  // calculating x using cramer's rule
     y = D2 / D;  // calculating y using cramer's rule
     z = D3 / D;  // calculating z using cramer's rule
    printf("%lf %lf %lf \n",x,y,z);
  }

  double phi = atan(sqrt(pow(x, 2) + pow(y, 2))/z);
  double theta = 0;
  if (x > 0) {
    theta = atan(y/x);
  }
  else if(x<0) {
    theta = atan(y/x) + 3.1415926;
  }
  else{
      theta = 3.1415926/2;
  }
  double r = sqrt(pow(x,2)+pow(y,2)+pow(z,2));
  
  double *angles = (double *) malloc(sizeof(double)*2);
  angles[0] = theta;
  angles[1] = phi;
  angles[2] = r;
  return angles;
}
