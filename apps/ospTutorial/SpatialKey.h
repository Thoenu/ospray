/**
 *  SpatialKey.h
 *  GlobeEngine
 *
 *  Created by Mathias Thšny on 27.12.11.
 *  Copyright (c) 2011 University of Zurich. All rights reserved.
 */

#ifndef GlobeEngine_SpatialKey_H
#define GlobeEngine_SpatialKey_H

#include <iostream>
#include <cassert>
#include <memory>

namespace ge {
    template <int D, class T> class SpatialTreeKey {
    public:
        SpatialTreeKey(){
            this->setLod(-1);
            for (int i=0;i<D;i++) {
                this->coord[i] = -1;
            }
        }
        
        SpatialTreeKey(short _lod){
            this->setLod(_lod);
            for (int i=0;i<D;i++) {
                this->coord[i] = -1;
            }
        }
        
        short getLod() const {  return this->lod;  };
        void setLod(short _in) {  this->lod = _in;  };
        
        const T* getCoord() const { return coord; };
        
        std::string toString() const {
            std::string res = std::to_string(this->lod);
            for (int i = 0; i<D; i++) {
                res += "/" + std::to_string(this->coord[i]);
            }
            return res;
        }
        
        std::string toStringPreAndPostfix(std::string _prefix, std::string _postfix) const {
            std::string res = _prefix;
            res += "/" + std::to_string(this->lod);
            for (int i = 0; i<D; i++) {
                res += "/" + std::to_string(this->coord[i]);
            }
            res += _postfix;
            return res;
        }
        
        friend std::ostream& operator<< (std::ostream &out, const SpatialTreeKey<D,T> &key) {
            out << "Key: (" << key.lod;
            for (int i=0;i<D;i++) {
                out << ","<< key.coord[i];
            }
            out << ")";
            //,rq: " << key.requestable <<" lo: " << key.loaded;
            return out;
        }
        
    protected:
        short lod;
        T coord[D];
    };
    
    template <class T> class SpatialTreeKey1 :  public SpatialTreeKey< 1, T>
    {
    public:
        SpatialTreeKey1(){
            this->setLod(-1);
            this->coord[0] = -1;
            this->childOrder = 0;
        }
        
        SpatialTreeKey1(short _lod) {
            this->setLod(_lod);
            this->coord[0] = -1;
            this->childOrder = 0;
        }
        
        SpatialTreeKey1(short _lod, unsigned int _x) {
            this->setLod(_lod);
            this->coord[0] = _x;
            this->childOrder = 0;
        }
        
        SpatialTreeKey1(short _lod, unsigned int _x, bool _order) {
            this->setLod(_lod);
            this->coord[0] = _x;
            this->childOrder = _order;
        }
        
        /* smaller with x coordinate priority. */
        bool operator<(const SpatialTreeKey1 &other)const{
            if(this->lod != other.lod)
                return this->lod < other.lod;
            return this->coord[0] < other.coord[0];
        }
        /* is equal is coordinates are equal */
        bool operator==(const SpatialTreeKey1 &other)const{
            return this->lod == other.lod && this->coord[0] == other.coord[0];
        }
        unsigned int getX() const {  return this->coord[0];  };
        void setX(unsigned int _in) {  this->coord[0] = _in; };
        
        /* 0 = first child, 1 = second child */
        bool isFirstChild() const {
            return !this->childOrder;
        }
        
        void setChildOrder(bool _in) {
            this->childOrder = _in;
        }
        
    protected:
        bool childOrder;
    };
    
    typedef SpatialTreeKey1<unsigned int> BinaryTreeKey;
    typedef SpatialTreeKey1<unsigned int> MultiwayTreeKey;
    typedef SpatialTreeKey1<char> SpatialKey1b;
    typedef SpatialTreeKey1<unsigned char> SpatialKey1ub;
    typedef SpatialTreeKey1<short> SpatialKey1s;
    typedef SpatialTreeKey1<unsigned short> SpatialKey1us;
    typedef SpatialTreeKey1<int> SpatialKey1i;
    typedef SpatialTreeKey1<unsigned int> SpatialKey1ui;
    
    /*
     *    Spatial keys for 2D data structures
     *    e.g. coordinates start bottom left by default. e.g:
     *
     *      coordinate structure     enum structure
     *      0/3 - 1/3 - 2/3 - 3/3    UL - UR
     *       |     |     |     |     |     |
     *      0/2 - 1/2 - 2/2 - 3/2    BL - BR
     *       |     |     |     |     index structure
     *      0/1 - 1/1 - 2/1 - 3/1    2 - 3
     *       |     |     |     |     |   |
     *      0/0 - 1/0 - 2/0 - 3/0    0 - 1
     *
     */
    template <class T> class SpatialTreeKey2 :  public SpatialTreeKey< 2, T>
    {
    public:
        SpatialTreeKey2(){
            this->setInitial(-1, -1, -1);
        }
        
        SpatialTreeKey2(short _lod) {
            this->setInitial(_lod, -1, -1);
        }
        
        SpatialTreeKey2(short _lod, unsigned int _x, unsigned int _y) {
            this->setInitial(_lod, _x, _y);
        }
        
        void setInitial(short _lod, unsigned int _x, unsigned int _y) {
            this->setLod(_lod);
            this->coord[0] = _x;
            this->coord[1] = _y;
        }
        
        // perform a deep copy of the spatial key
        SpatialTreeKey2(const std::shared_ptr<SpatialTreeKey2> _copy){
            this->setLod(_copy->getLod());
            this->coord[0] = _copy->getX();
            this->coord[1] = _copy->getY();
            
            this->parent = _copy->parent;
            for(int i=0;i<4;i++){
                this->childs[i] = _copy->childs[i];
            }
        }
        
        std::shared_ptr< SpatialTreeKey2<T> > getParentKey() const {
            return this->parent;
        }
        
		std::shared_ptr< SpatialTreeKey2<T> > getChildKey(int _idx) const {
            if(!childs[_idx]){
                switch (_idx) {
                    case 0:{
                        this->childs[_idx] = std::make_shared<SpatialTreeKey2<T> >(this->lod+1,this->coord[0]*2, this->coord[1]*2);
                        break; }
                    case 1:{
                        this->childs[_idx] = std::make_shared<SpatialTreeKey2<T> >(this->lod+1,this->coord[0]*2 + 1, this->coord[1]*2);
                        break; }
                    case 2:{
                        this->childs[_idx] = std::make_shared<SpatialTreeKey2<T> >(this->lod+1,this->coord[0]*2, this->coord[1]*2 + 1);
                        break; }
                    case 3:{
                        this->childs[_idx] = std::make_shared<SpatialTreeKey2<T> >(this->lod+1,this->coord[0]*2 + 1, this->coord[1]*2 + 1);
                        break; }
                    default:
                        std::cout << "Child key request out of range" << std::endl;
                        assert(false);
                        break;
                }
            }
            return childs[_idx];
        }
        
        int getChildKeyInDirectionTo(std::shared_ptr< SpatialTreeKey2<T> > _key) {
            this->getChildKey(getChildIdxInDirectionTo(_key));
        }
            
        int getChildIdxInDirectionTo(std::shared_ptr< SpatialTreeKey2<T> > _key) {
            double powOfDiff = pow(2.0f, _key->getLod() - this->getLod());
            int keysProj[2];
            for (int i = 0; i < 2; i++){
                keysProj[i] = this->getCoord()[i] * powOfDiff;
            }
            int sidelenghtOfChild = (int)(powOfDiff / 2.0f);
            if (keysProj[0] <= _key->getX() && _key->getX() < keysProj[0] + sidelenghtOfChild) {
                if (keysProj[1] <= _key->getY() && _key->getY() < keysProj[1] + sidelenghtOfChild) {
                    return 0;
                }
                else {
                    return 2;
                }
            }
            else {
                if (keysProj[1] <= _key->getY() && _key->getY() < keysProj[1] + sidelenghtOfChild) {
                    return 1;
                }
                else {
                    return 3;
                }
            }
        }
        
        /* smaller with x coordinate priority. And priority for stuff in view frustum. */
        bool operator<(const SpatialTreeKey2 &other)const{
            if(this->inViewFrustum && !other.inViewFrustum)
                return this->inViewFrustum;
            if(!this->inViewFrustum && other.inViewFrustum)
                return this->inViewFrustum;
            
            if(this->lod != other.lod)
                return this->lod < other.lod;
            if(this->coord[0] != other.coord[0])
                return this->coord[0] < other.coord[0];
            return this->coord[1] < other.coord[1];
        }
        
        /* is equal is coordinates are equal */
        bool operator==(const SpatialTreeKey2 &other)const{
            return this->lod == other.lod && this->coord[0] == other.coord[0] && this->coord[1] == other.coord[1];
        }
        
        unsigned int getX() const {  return this->coord[0];  };
        void setX(unsigned int _in) {  this->coord[0] = _in; };
        unsigned int getY() const {  return this->coord[1]; };
        void setY(unsigned int _in) { this->coord[1] = _in; };
        
    private:
        std::shared_ptr< SpatialTreeKey2<T> > parent;
        std::shared_ptr< SpatialTreeKey2<T> > childs[4];
    };
    
    typedef SpatialTreeKey2<char> SpatialKey2b;
    typedef SpatialTreeKey2<unsigned char> SpatialKey2ub;
    typedef SpatialTreeKey2<short> SpatialKey2s;
    typedef SpatialTreeKey2<unsigned short> SpatialKey2us;
    typedef SpatialTreeKey2<int> SpatialKey2i;
    typedef SpatialTreeKey2<unsigned int> SpatialKey2ui;
    
}
#endif
