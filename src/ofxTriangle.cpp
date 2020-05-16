#include "ofxTriangle.h"

#ifdef USE_OPENCV
void ofxTriangle::triangulate(ofxCvBlob &cvblob, int resolution) {
	triangulate(cvblob.pts, resolution);
}
#endif

void ofxTriangle::triangulate(vector<glm::vec3> contour, int resolution) {
    int bSize = contour.size();
    float maxi = min(resolution, bSize);
	
    Delaunay::Point tempP;
    vector<Delaunay::Point> v;
	
    for(int i = 0; i < maxi; i++) {
		int indx = ofMap(i, 0, maxi, 0, bSize);
        tempP[0] = contour[indx].x;
        tempP[1] = contour[indx].y;
		
        v.push_back(tempP);
    }
	
    delobject = new Delaunay(v);
    delobject->Triangulate();
    
    Delaunay::fIterator fit;
    for ( fit = delobject->fbegin(); fit != delobject->fend(); ++fit ) {
        int pta = delobject->Org(fit);
        int ptb = delobject->Dest(fit);
        int ptc = delobject->Apex(fit);
        
        int pta_id = (int)ofMap(pta, 0, maxi, 0, bSize);
        int ptb_id = (int)ofMap(ptb, 0, maxi, 0, bSize);
        int ptc_id = (int)ofMap(ptc, 0, maxi, 0, bSize);
		
        glm::vec3 tr[3];
        tr[0] = glm::vec3(contour[pta_id].x, contour[pta_id].y, 0);
        tr[1] = glm::vec3(contour[ptb_id].x, contour[ptb_id].y, 0);
        tr[2] = glm::vec3(contour[ptc_id].x, contour[ptc_id].y, 0);
		
        if( isPointInsidePolygon(&contour[0], contour.size(), getTriangleCenter(tr) ) ) {
            ofxTriangleData td;
            td.a = glm::vec3(tr[0].x, tr[0].y, 0);
            td.b = glm::vec3(tr[1].x, tr[1].y, 0);
            td.c = glm::vec3(tr[2].x, tr[2].y, 0);
            
            td.area = delobject->area(fit);
            
            triangles.push_back(td);
			
            nTriangles++;
        }
    }
	
    delete delobject;
}

void ofxTriangle::clear() {
    triangles.clear();
    nTriangles = 0;
}

glm::vec3 ofxTriangle::getTriangleCenter(glm::vec3 *tr) {
    float c_x = (tr[0].x + tr[1].x + tr[2].x) / 3;
    float c_y = (tr[0].y + tr[1].y + tr[2].y) / 3;
    
    return glm::vec3(c_x, c_y, 0);
}

bool ofxTriangle::isPointInsidePolygon(glm::vec3 *polygon,int N, glm::vec3 p) {
    int counter = 0;
    int i;
    double xinters;
    glm::vec3 p1, p2;
    
    p1 = polygon[0];
    
    for (i=1;i<=N;i++) {
        p2 = polygon[i % N];
        if (p.y > MIN(p1.y,p2.y)) {
            if (p.y <= MAX(p1.y,p2.y)) {
                if (p.x <= MAX(p1.x,p2.x)) {
                    if (p1.y != p2.y) {
                        xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
                        if (p1.x == p2.x || p.x <= xinters){
                            counter++;
						}
                    }
                }
            }
        }
        p1 = p2;
    }
    
	return counter % 2 != 0;
}

void ofxTriangle::draw() {
    for (int i=0; i<nTriangles; i++) {
        ofDrawTriangle(triangles[i].a.x, triangles[i].a.y,
                       triangles[i].b.x, triangles[i].b.y,
                       triangles[i].c.x, triangles[i].c.y);
    }
}

void ofxTriangle::draw(float x, float y) {
    ofPushMatrix();
    ofTranslate(x, y, 0);
	draw();
    ofPopMatrix();
}

void ofxTriangle::draw(float x, float y, bool fill) {
    ofPushMatrix();
    ofTranslate(x, y, 0);
    
    if (fill) {
        ofFill();
    } else {
        ofNoFill();
    }
    draw();
    ofPopMatrix();
}

void ofxTriangle::draw(float r, float g, float b) {
    ofFill();
	
    for (int i=0; i<nTriangles; i++){
        ofSetColor(r,g,b);
        ofDrawTriangle(triangles[i].a.x, triangles[i].a.y,
                       triangles[i].b.x, triangles[i].b.y,
                       triangles[i].c.x, triangles[i].c.y);
    }
}

void ofxTriangle::draw(float x, float y, float r, float g, float b) {
    ofPushMatrix();
    ofTranslate(x, y, 0);
    
    ofFill();
	
    for (int i=0; i<nTriangles; i++){
        ofSetColor(r,g,b);
        ofDrawTriangle(triangles[i].a.x, triangles[i].a.y,
                       triangles[i].b.x, triangles[i].b.y,
                       triangles[i].c.x, triangles[i].c.y);
    }
    ofPopMatrix();
}

void ofxTriangle::draw(float x, float y, float r, float g, float b, float alpha) {
    ofPushMatrix();
    ofTranslate(x, y, 0);
    
    ofFill();
	
    for (int i=0; i<nTriangles; i++){
        ofSetColor(r,g,b,alpha);
        ofDrawTriangle(triangles[i].a.x, triangles[i].a.y,
                       triangles[i].b.x, triangles[i].b.y,
                       triangles[i].c.x, triangles[i].c.y);
    }
    ofPopMatrix();
}

void ofxTriangle::draw(float x, float y, float r, float g, float b, float alpha, bool fill) {
    ofPushMatrix();
    ofTranslate(x, y, 0);
    
    if (fill) {
        ofFill();
    } else {
        ofNoFill();
    }
    
    for (int i=0; i<nTriangles; i++){
        ofSetColor(r,g,b,alpha);
        ofDrawTriangle(triangles[i].a.x, triangles[i].a.y,
                       triangles[i].b.x, triangles[i].b.y,
                       triangles[i].c.x, triangles[i].c.y);
    }
    ofPopMatrix();
}
