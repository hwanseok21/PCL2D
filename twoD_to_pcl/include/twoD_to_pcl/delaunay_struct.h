struct vect { // 벡터 구조체, 프로젝션 벡터나 코사인, 내적 외적등을 편리하게 구하기 위하여 구현
    double x;
    double y;
    vect() { x = y = 0; }
    vect(double x, double y) {
        this->x = x;
        this->y = y;
    }
    const double dist() const { // 거리
        return sqrt(x * x + y * y);
    }
    const double inner(const vect &a) const { // 내적
        return x * a.x + y * a.y;
    }
    const double cross(const vect &a) const { // 외적(determinant)
        return x * a.y - y * a.x;
    }
    const vect operator+ (const vect &a) const { // 벡터의 합
        return vect(x + a.x, y + a.y);
    }
    const vect operator- (const vect &a) const { // 벡터의 차
        return vect(x - a.x, y - a.y);
    }
    const vect operator* (const double &a) const { // 스칼라 곱
        return vect(a * x, a * y);
    }
    const vect proj(const vect &a) const { // projection vector
        return *this * (inner(*this) / inner(a));
    }
    const double get_cos(const vect &a) const { // 두 벡터의 코사인
        return inner(a) / (dist() * a.dist());
    }
};

struct edg { // edge 구조체, 말 그대로 변에 대한 구조체
    int a;
    int b;
    edg() { a = b = 0; }
    edg(int a, int b) {
        if(a < b) {
            this->a = a;
            this->b = b;
        }
        else {
            this->a = b;
            this->b = a;
        }
    }
    const bool operator== (const edg &x) const {
        return a == x.a && b == x.b;
    }
    const bool operator< (const edg &x) const {
        if(a == x.a) return b < x.b;
        return a < x.a;
    }
};

struct tri { // triangle 구조체, 말 그대로 삼각형에 대한 구조체
    int a;
    int b;
    int c;
    tri() { a = b = c = 0;}
    tri(int a, int b, int c) {
        this->a = a;
        this->b = b;
        this->c = c;
    }
};

bool is_circum(tri cur, int i, vector<vect> &point) { // 외접원안에 점이 들어오는지 확인

    double ccw = (point[cur.b] - point[cur.a]).cross(point[cur.c] - point[cur.a]);

    double adx=point[cur.a].x-point[i].x, ady=point[cur.a].y-point[i].y,
    bdx=point[cur.b].x-point[i].x, bdy=point[cur.b].y-point[i].y,
    cdx=point[cur.c].x-point[i].x, cdy=point[cur.c].y-point[i].y,
    bdxcdy = bdx * cdy, cdxbdy = cdx * bdy,
    cdxady = cdx * ady, adxcdy = adx * cdy,
    adxbdy = adx * bdy, bdxady = bdx * ady,
    alift = adx * adx + ady * ady,
    blift = bdx * bdx + bdy * bdy,
    clift = cdx * cdx + cdy * cdy;
    double det = alift * (bdxcdy - cdxbdy)
    + blift * (cdxady - adxcdy)
    + clift * (adxbdy - bdxady);
    
    if(ccw > 0) return det >= 0;
    else return det <= 0;
}

