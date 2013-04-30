#ifndef L3_EXTERNAL_INTERFAE_H
#define L3_EXTERNAL_INTERFAE_H

#include <boost/bind.hpp>

namespace L3
{
namespace Visualisers
{
    struct ExternalInterface : glv::TextView 
    {
        ExternalInterface( glv::Rect rect ) : glv::TextView( rect )
        {
            visible = false;
            this->maximize(); 
            this->disable(glv::Visible);

            //current_pos = 0; 
        }

        bool visible;

        //int current_pos;

        bool onEvent( glv::Event::t e, glv::GLV& g)
        {
            switch( e )
            {
                case glv::Event::MouseDown:
                case glv::Event::MouseUp:
                case glv::Event::MouseDrag:
                    return false;
            }
            
            const glv::Keyboard& k = g.keyboard();
            int key = k.key();

            //std::cout << k.shift() << std::endl;

            // Special switch key
            if (key == 96)
            {
                visible ? this->disable(glv::Visible) : this->enable(glv::Visible);
                visible = !visible; 
                return true;
            }

            if ( !visible )
                return true;

            // Insert the text
            mText.insert(mPos, 1, key); 
            setValue(mText);
            setPos( mPos+1 );

            //mText.insert(current_pos, 1, key); 
            //setValue(mText);
            //current_pos++;

            switch(key)
            {
                case glv::Key::Delete:
                    if(selected()) deleteSelected();
                    else if(validPos()){
                        deleteText(mPos-1, 1);
                        setPos(mPos-1);
                    }
                    return false;

                case glv::Key::BackSpace:
                    if(selected()) deleteSelected();
                    else if(mText.size()){
                        deleteText(mPos, 1);
                        setPos(mPos);
                    }
                    return false;

                case glv::Key::Left:
                    if(k.shift()) select(mSel-1);
                    else setPos(mPos-1);
                    return false;

                case glv::Key::Right:
                    if(k.shift()) select(mSel+1);
                    else setPos(mPos+1);
                    return false;

                case glv::Key::Down: 
                    //setPos(mText.size()); 
                    return false;

                case glv::Key::Up:   
                    //setPos(0); 
                    return false;

                case glv::Key::Enter:
                case glv::Key::Return:
                    std::cout << mText << std::endl;
                    return false;
            }

            return true;
        }

        void onDraw( glv::GLV& g){

            if(++mBlink==40) mBlink=0; // update blink interval

            float padX = mPadX;
            float padY = 4;
            float addY =-4;//was -2     // subtraction from top since some letters go above cap

            float tl = mPos * font().advance('M') + padX;
            //  float tr = tl + font().advance('M');
            float tt = addY + padY;
            float tb = tt + fabs(font().descent()+font().cap()) - addY;
            float strokeWidth = 1;

            // draw selection
            if(selected()){
                float sl, sr;
                if(mSel>0){
                    sl = tl;
                    sr = sl + mSel*font().advance('M');
                }
                else{
                    sr = tl;
                    sl = sr + mSel*font().advance('M');
                }
                glv::draw::color(colors().selection);
                glv::draw::rectangle(sl, tt, sr, tb);
            }

            // draw cursor
            if(mBlink<20 && enabled(glv::Focused)){
                glv::draw::color(colors().text);
                glv::draw::shape(glv::draw::Lines, tl, tt, tl, tb);
            }

            std::stringstream ss;

            //ss << ">> " << mText;
            //std::cout << ss.str() << std::endl;

            glv::draw::lineWidth(strokeWidth);
            glv::draw::color(colors().text);
            font().render(mText.c_str(), padX, padY);
            //font().render(ss.str().c_str(), padX, padY);
        }

    };

}
}

#endif

