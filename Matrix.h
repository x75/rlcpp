 #ifndef MATRIX_H
#define MATRIX_H

class Matrix {
    public:
        Matrix() ;
        Matrix( int  );
        Matrix( int , int );
        Matrix( int , int, double* );
        ~Matrix();

        void setSize( int ) ;
        void setSize( int, int ) ;
        int getSize() ;
    
        double * getPtr( int i ) ;
        double * getPtr() ;
    
        double get( int ) const ;
        void set( int, double );
        void add( int, double );
        double get( int, int ) const ;
        void set( int, int , double );
        void add( int, int , double );
        //~ Matrix multiply( Matrix, int, int );

    private:
        int _nRows, _nCols, _size, _id;
        double* _data;
};
#endif // MATRIX_H
