import cv2
import numpy
_thickness = 2

def recurseTriangle(triLeft, im, pts):
	p1, p2, p3 = pts
	#draw triangle between point set
	cv2.line(im, p1, p2, (255,255,255), thickness=_thickness)
	cv2.line(im, p2, p3, (255,255,255), thickness=_thickness)
	cv2.line(im, p3, p1, (255,255,255), thickness=_thickness)

	#show image
	if triLeft > 3:
		cv2.imshow("Triangles", im)
		k = cv2.waitKey(1) & 0xFF
		#end early
		if k == 27:
			triLeft = 0

	#if there are triangles left to draw, draw
	if triLeft>0:
		s1, s2, s3 = triangle(p1, p2, p3)
		recurseTriangle(triLeft-1, im, s1)
		recurseTriangle(triLeft-1, im, s2)
		recurseTriangle(triLeft-1, im, s3)
#end

def triangle(p1, p2, p3):
	#basically return set of inner triangles
	m1	=	((p1[0]+p2[0])//2, (p1[1]+p2[1])//2 )
	m2	=	((p2[0]+p3[0])//2, (p2[1]+p3[1])//2 )
	m3	=	((p3[0]+p1[0])//2, (p3[1]+p1[1])//2 )

	s1	=	(p1, m1, m3)
	s2	=	(m1, p2, m2)
	s3	=	(m3, m2, p3)

	return s1, s2, s3
#end

if __name__ == "__main__":
	maxTri=6		#number of iterations to make after 1st triangle

	dim = 2000
	im = numpy.zeros((dim, dim))
	p1 = (0  , dim)
	p2 = (dim, dim)
	p3 = (dim//2 , 0  )
	points = (p1, p2, p3)

	cv2.namedWindow("Triangles", cv2.WINDOW_NORMAL)
	recurseTriangle(maxTri, im, points)

	cv2.imshow("Triangles", im)
	cv2.waitKey(0)
#end