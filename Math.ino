
/*************************************************
** FILE : Math
** This file contains all the math helper functions.
** These functions help compute common functions
** which are not defined in the standard libraries.
**************************************************/


/*************************************************
** Vector_Dot_Product
** Computes the dot product of two vectors 
**   scalar = Vector_Dot_Product( v1[], v2[] )
**   scalar = <v1,v2>
*/
float Vector_Dot_Product(const float v1[3], const float v2[3])
{
  float result = 0;
  for(int c = 0; c < 3; c++) { result += v1[c] * v2[c]; }
  return result; 
}


/*************************************************
** Vector_Cross_Product
** Computes the cross product of two vectors 
**   Vector_Cross_Product( v1[], v2[], out[] )
**   out[] = v1[] x v2[]
*/
void Vector_Cross_Product( const float v1[3], const float v2[3], float out[3] )
{
  out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}


/*************************************************
** Vector_Scale
** Multiply a vector by a scalar (element-wise)
**   Vector_Scale( v[], scalar, out[] )
**   out[] = v[] .* scalar
*/
void Vector_Scale( const float v[3], float scale, float out[3] )
{
  for(int c = 0; c < 3; c++) { out[c] = v[c] * scale; }
}


/*************************************************
** Vector_Add
** Adds two vectors element-wise
**   Vector_Add( out[], v1[], v2[] )
**   out[] = v1[] + v2[] 
*/
void Vector_Add( const float v1[3], const float v2[3], float out[3] )
{
  for(int c = 0; c < 3; c++) { out[c] = v1[c] + v2[c]; }
}


/*************************************************
** Matrix_Multiply
** Multiply two 3x3 matrices
**   Matrix_Multiply( m1[][], m2[][], out[][] )
**   out = m1 * m2 
*/
void Matrix_Multiply(const float m1[3][3], const float m2[3][3], float out[3][3])
{
  for(int x = 0; x < 3; x++)  // rows
  {
    for(int y = 0; y < 3; y++)  // columns
    {
      out[x][y] = m1[x][0] * m2[0][y] + m1[x][1] * m2[1][y] + m1[x][2] * m2[2][y];
    }
  }
}


/*************************************************
** Matrix_Vector_Multiply
** Multiply 3x3 matrix with 3x1 vector
** Outputs a 3x1 vector
**   Matrix_Vector_Multiply( m[][], v[], out[] ) 
**   out = m * v
*/
void Matrix_Vector_Multiply(const float m[3][3], const float v[3], float out[3])
{
  for(int x = 0; x < 3; x++) { out[x] = m[x][0] * v[0] + m[x][1] * v[1] + m[x][2] * v[2]; }
}


/*************************************************
** f_asin
** A faster arcsin
*/
float f_asin( float x ) 
{
  float negate = float(x < 0);
  x = abs(x);
  float ret = -0.0187293;
  ret *= x;
  ret += 0.0742610;
  ret *= x;
  ret -= 0.2121144;
  ret *= x;
  ret += 1.5707288;
  ret = 3.14159265358979*0.5 - sqrt(1.0 - x)*ret;
  return ret - 2 * negate * ret;
}


/************************************************* 
** f_atan2
** A faster arctan2
*/
float f_atan2( float y, float x )
{
  float t0, t1, t2, t3, t4;

  t3 = abs(x);
  t1 = abs(y);
  t0 = max(t3, t1);
  t1 = min(t3, t1);
  t3 = float(1) / t0;
  t3 = t1 * t3;

  t4 = t3 * t3;
  t0 = - float(0.013480470);
  t0 = t0 * t4 + float(0.057477314);
  t0 = t0 * t4 - float(0.121239071);
  t0 = t0 * t4 + float(0.195635925);
  t0 = t0 * t4 - float(0.332994597);
  t0 = t0 * t4 + float(0.999995630);
  t3 = t0 * t3;

  t3 = (abs(y) > abs(x)) ? float(1.570796327) - t3 : t3;
  t3 = (x < 0) ?  float(3.141592654) - t3 : t3;
  t3 = (y < 0) ? -t3 : t3;

  return t3;
}









