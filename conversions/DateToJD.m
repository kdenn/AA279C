function JD = DateToJD(varargin)
    % Convert a date to Julian Date
    %{
    ---------------------------------------------------------------
    INPUT:
        Call 1: DateToJD(year,month,day,hour,min,sec)
        Call 2: DateToJD(year,day,sec)
    ---------------------------------------------------------------
    OUTPUT:
        JD:     scalar, Julian Date
    ---------------------------------------------------------------
    AUTHOR: Corinne Lippe, 2019
    ---------------------------------------------------------------
    %}
    if nargin == 6
        year = varargin{1};
        month = varargin{2};
        day = varargin{3};
        hour = varargin{4};
        min = varargin{5};
        sec = varargin{6};
    elseif nargin == 3
        year = varargin{1};
        day = varargin{2};
        sec = varargin{3};
        month = 0;
        hour = 0;
        min = 0;
    else
        error('Wrong number of input arguments')
    end

    JD = datenum(year,month,day,hour,min,sec)+1721058.5;

end