/*	Graphics Library of Views (GLV) - GUI Building Toolkit
	See COPYRIGHT file for authors and license information */

#include "glv_grid.h"

namespace glv{

Grid::Grid(const Rect& r, double rangeMin, double rangeMax, double majorDist, int minorDiv)
:	View(r), mEqualize(false)
{
	showAxis(true);
	showGrid(true);
	showNumbering(false);
	range(rangeMin, rangeMax);
	major(majorDist); minor(minorDiv);
	for(int i=0; i<DIM; ++i){
		mVel[i]=0;
		lockScroll(false, i);
		lockZoom(false, i);
	}
	mVelW=0;
}

// add grid lines to vertex buffer, returns number of lines added
int Grid::addGridLines(int i, double dist, GraphicsData& gd){

	float  p = gridToPix(i, floor(interval(i).min(), dist));				
	float dp = gridToPix(i, dist) - gridToPix(i, 0);
	if(fabs(dp) < 2) return 0;

	double nr = interval(i).diameter()/dist;
	int n = nr;
	int j = (i+1)%DIM;
	n += 2; // safety measure for edge cases

	for(int idx=0; idx<n; ++idx){
		float v[2][2] = {
			{p+idx*dp, 0},
			{p+idx*dp, extentVector[j]}
		};
		gd.addVertex2(v[0][i], v[0][j], v[1][i], v[1][j]);
	}
	return n;
}

void Grid::onAnimate(double dt){
//	for(int i=0; i<DIM; ++i){
//		if(mVel[i] != 0) interval(i).translate(mVel[i]);
//	}
//	if(mVelW != 0) zoomOnMousePos(mVelW, g.mouse);
}

void Grid::onDraw(GLV& g){

	for(int i=0; i<DIM; ++i){
		if(!mLockScroll[i] && mVel[i] != 0) interval(i).translate(mVel[i]);
	}
	if(mVelW != 0) zoomOnMousePos(mVelW, g.mouse());

	using namespace glv::draw;
	GraphicsData& gd = g.graphicsData();

	lineWidth(1);

	// Draw minor lines
	gd.reset();
	color(colors().border.mix(colors().back, 14./16));
	for(int i=0; i<DIM; ++i){
		if(mShowGrid[i] && mMinor[i]>1){
			addGridLines(i, mMajor[i]/mMinor[i], gd);
		}
	}
	paint(Lines, &gd.vertices2()[0], gd.vertices2().size());


	// Draw major lines / numbering
	gd.reset();
	for(int i=0; i<DIM; ++i){
		if(mShowGrid[i] || mShowNumbering[i]){
			int b = gd.vertices2().size();
			int numMajLines = addGridLines(i, mMajor[i], gd);

			if(mShowNumbering[i]){
			
				color(colors().border);
			
				// iterate through major lines for this dimension
				for(int j=b; j<b+numMajLines*2; j+=2){
					double p = gd.vertices2()[j].elems[i];
					double v[] = { 
						i ?   4 : p+4,
						i ? p+4 : h-(4+font().cap())
					};
					double val = pixToGrid(i, p);
					if(fabs(val) < 1e-5) val=0;
					char buf[16];
					GLV_SNPRINTF(buf, sizeof(buf), "%.3g", val);
					font().render(g.graphicsData(1), buf, v[0], v[1]);
				}
			}

			// remove lines if not showing grid
			if(!mShowGrid[i]) gd.vertices2().size(b);
		}
	}
	color(colors().border.mix(colors().back, 10./16));
	paint(Lines, &gd.vertices2()[0], gd.vertices2().size());

	// Draw axes
	color(colors().border.mix(colors().back, 0./4));
	if(mShowAxis[0] && interval(1).contains(0)){
		float p = gridToPix(1, 0);
		shape(Lines, 0, p, w, p);
	}
	if(mShowAxis[1] && interval(0).contains(0)){
		float p = gridToPix(0, 0);
		shape(Lines, p, 0, p, h);
	}
//	if(mEqualize){ // NOTE: this always works when called from draw loop
//		w>=h	? interval(0).diameter(interval(1).diameter()*w/h)
//				: interval(1).diameter(interval(0).diameter()*h/w);
//	}
}

// Adds 1 to bool array treated like a binary number
template <int N>
void nextBoolArrayState(bool * arr){
	int state = 0;
	for(int i=0; i<N; ++i){
		state |= int(arr[i])<<i;
	}
	++state;
	for(int i=0; i<N; ++i){
		arr[i] = (state>>i)&1;
	}
}

bool Grid::onEvent(Event::t e, GLV& g){
//	printf("[% 6.3f, % 6.3f], [% 6.3f, % 6.3f]\n",
//		interval(0).min(), interval(0).max(), interval(1).min(), interval(1).max());

	const Mouse& m = g.mouse();
	const Keyboard& k = g.keyboard();
	switch(e){			
		case Event::MouseDown:
			return false;

		case Event::MouseDrag:
			if(m.left()){
				if(!mLockScroll[0]) interval(0).translate(-pixToGridMul(0, m.dx()));
				if(!mLockScroll[1]) interval(1).translate( pixToGridMul(1, m.dy()));
			}
			if(m.right()){
				zoomOnMousePos(m.dy()*0.01, m);			
			}
			return false;

		case Event::KeyDown:
			//printf("%c %d\n", k.key(), k.key());
			//if(k.shift()){
				switch(k.key()){
					case 'a': mVel[0] =-pixToGridMul(0,8); break;
					case 'd': mVel[0] = pixToGridMul(0,8); break;
					case 'x': mVel[1] =-pixToGridMul(0,8); break;
					case 'w': mVel[1] = pixToGridMul(0,8); break;
					case 'e': mVelW =-0.04; break;
					case 'c': mVelW = 0.04; break;
					case 's': origin(); break;
					case 'g': nextBoolArrayState<DIM>(mShowGrid); break;
					case 'b': nextBoolArrayState<DIM>(mShowAxis); break;
					case 'n': nextBoolArrayState<DIM>(mShowNumbering); break;
					case '+': for(int i=0;i<DIM;++i) interval(i).scale(0.5); break;
					case '-': for(int i=0;i<DIM;++i) interval(i).scale(2.0); break;
//					case 'p': mPolarGrid ^= 1; break;
					default: return true;
				}
				return false;
//			}
//			else{
//				return true;
//			}
		
		case Event::MouseUp:	return false;
		case Event::KeyUp:
			switch(k.key()){
				case 'a':
				case 'd': mVel[0] = 0; break;
				case 'x':
				case 'w': mVel[1] = 0; break;
				case 'e':
				case 'c': mVelW = 0; break;
				default: return true;
			}
			return false;

		default: return true;
	}
}

void Grid::onResize(space_t dx, space_t dy){
	if(mEqualize){
		// always adjust x interval, unless there is a better way...
		interval(0).diameter(interval(1).diameter()*(h>0 ? w/h : 1));
//		w>=h	? interval(0).diameter(interval(1).diameter()*w/h)
//				: interval(1).diameter(interval(0).diameter()*h/w);
//		float pw = w-dx;
//		float ph = h-dy;
	}
}

Grid& Grid::zoom(double amt, double x, double y){
	// change diameter around arbitrary point in interval
	// 1. translate center of interval to inflection point
	// 2. scale diameter
	// 3. translate center of interval back to scaled original					
	double scale = pow(2, amt);
	double gs[] = { x, y };
	
	for(int i=0; i<DIM; ++i){
		if(mLockZoom[i]) continue;

		interval_t iv = interval(i);
		float t = gs[i] - iv.center();
		iv.translate(t);
		iv.diameter(iv.diameter()*scale);
		iv.translate(-t*scale);
		interval(i) = iv;				
	}
	return *this;
}

void Grid::zoomOnMousePos(double amt, const Mouse& m){
	float px = m.xRel(m.button());	// pixel x coord
	float py = m.yRel(m.button());	// pixel y coord					
	zoom(amt, pixToGrid(0, px), pixToGrid(1, py));
}

Grid& Grid::origin(){
	for(int i=0; i<DIM; ++i){ interval(i).center(0); }
	return *this;
}


template <int N, class T>
void setBool(T * arr, T v, int idx){
	if(idx != -1)	arr[idx] = v;
	else			for(int i=0; i<N; ++i){ arr[i] = v; }
}

Grid& Grid::lockScroll(bool v, int dim){
	setBool<DIM>(mLockScroll, v, dim);
	return *this;
}

Grid& Grid::lockZoom(bool v, int dim){
	setBool<DIM>(mLockZoom, v, dim);
	return *this;
}

Grid& Grid::minor(int v, int dim){
	setBool<DIM>(mMinor, v, dim);
	return *this;
}

Grid& Grid::major(double v, int dim){
	setBool<DIM>(mMajor, v, dim);
	return *this;
}

Grid& Grid::showAxis(bool v, int dim){
	setBool<DIM>(mShowAxis, v, dim);
	return *this;
}

Grid& Grid::showGrid(bool v, int dim){
	setBool<DIM>(mShowGrid, v, dim);
	return *this;
}

Grid& Grid::showNumbering(bool v, int dim){
	setBool<DIM>(mShowNumbering, v, dim);
	return *this;
}

Grid& Grid::range(double min, double max, int dim){
	if(dim != -1)	interval(dim).endpoints(min,max);
	else			for(int i=0; i<DIM; ++i){ interval(i).endpoints(min,max); }
	return *this;
}


void Grid::pushGrid(){
	double tx = gridToPix(0, 0);
	double ty = gridToPix(1, 0);
	double sx = gridToPix(0, interval(0).min()+1);
	double sy = gridToPix(1, interval(1).min()+1);
	draw::push();
	draw::translate(tx, ty);
	draw::scale(sx,sy-h);
}

void Grid::popGrid(){
	draw::pop();
}

} // glv::
