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
        }

        bool visible;

        bool onEvent( glv::Event::t e, glv::GLV& g)
        {
            const glv::Keyboard& k = g.keyboard();
            int key = k.key();

            //if(isprint(key)){
				//deleteSelected();
				//mText.insert(mPos, 1, k.key()); setValue(mText);
				//setPos(mPos+1);
				//return false;
			//}
	
            mText.insert(mPos, 1, k.key()); 
            setValue(mText);

            switch(key)
            {
                case 96: 
                    visible ? this->disable(glv::Visible) : this->enable(glv::Visible);
                    visible = !visible;

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

                case glv::Key::Down: setPos(mText.size()); return false;
                case glv::Key::Up:   setPos(0); return false;

                case glv::Key::Enter:
                case glv::Key::Return:
                                     return false;
            }

            return true;
        }






        //bool onEvent( glv::Event::t e, glv::GLV& g)
        //{
        //switch(e)
        //{
        //case glv::Event::MouseDown:
        //case glv::Event::MouseDrag:
        //case glv::Event::KeyDown:

        //if (g.keyboard().key() == 96) 
        //{
        //visible ? this->disable(glv::Visible) : this->enable(glv::Visible);
        //visible = !visible;
        //}
        //if (g.keyboard().key() == glv::Key::Enter)
        //std::cout << mText << std::endl;
        //else
        //glv::TextView::onEvent( e, g );


        //}

        //return true;
        //}

    };


}
}

#endif

