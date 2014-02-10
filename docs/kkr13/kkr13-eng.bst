%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Styl bibliografii "kkr13-eng.bst" jest udostepniony na potrzeby 
%   13 Krajowej Konfrencji Robotyki.
%
% Autor: Bogdan Kreczmer (e-mail: bogdan.kreczmer@pwr.wroc.pl)
% Data utworzenia:       16.12.2007
% Ostatnia modyfikacja:  30.05.2010
% Data ostatniej modyfikacji: 27.06.2010
%
% Modyfikacje:
% 27.06.2010 - wymuszone zostalo tworzenie inicjalow imion dla
%              cytowanej pozycji.
%
%
% Niniejszy styl jest oparty na stylu "plplain.bst". Najistotniejsza
% roznica wynika z koniecznosci umozliwienia cytowania adresow stron
% internetowych i lokalizacji ich na koncu bibliografii,
%
% Spelnienie powyzszych wymagan wymusilo przyjecie dodatkowych zalozen
% co do struktury plikow typu ".bib".
% Zalozenia te to:
%  -  Dane dotyczace stron internetowych sa umieszczane z
%     zastosowaniem nowej niestandardowej pozycji "@Website".
%     Przyklad jej uzycia w pliku ".bib":
%
% @Website{AD1871,
%  author = {Analog Devices},
%  title =  {AD1871 -- Stereo Audio},
%  url = {http://www.analog.com/prod/0,,760\_789\_AD1871\%2C00.html}
% }
%
%  -  W stosunku do wersji kkr13.bst nie trzeba uzywac
%     dodatkowego atrybutu "lang" w opisie bibliograficznym.
%     Jego uzycie nie jest bledem i jest ignorowane.
%
% Wszelkie zgloszenia problemow i zapytania odnoszace sie do udostepnionego
% stylu nalezy kierowac na adres: bogdan.kreczmer@pwr.wroc.pl
%
% Copyright (C) 2007-2010, wszystkie prawa zastrzezone.
% Kopiowanie i wykorzystywanie niniejszego pliku jest dozwolone
% pod warunkiem, ze:
%  1. do niniejszego pliku nie zostana wprowadzone zadne zmiany lub
%  2. po wprowadzeniu zmian zostanie zmieniona nazwa tego pliku.
%     Nowy plik nie powinien miec zadnej z nazw: kkr13-utf8.bst,
%     kkr13-latin2.bst, kkr13-ciach.bst, kkr13-dos.bst, kkr13.bst,
%     kkr13-eng.bst, jak tez zadnej z nazw plikow, ktore zostaly
%     udostepnione na potrzeby poprzednich KKR.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


MACRO {jan} {"m01"}

MACRO {feb} {"m02"}

MACRO {mar} {"m03"}

MACRO {apr} {"m04"}

MACRO {may} {"m05"}

MACRO {jun} {"m06"}

MACRO {jul} {"m07"}

MACRO {aug} {"m08"}

MACRO {sep} {"m09"}

MACRO {oct} {"m10"}

MACRO {nov} {"m11"}

MACRO {dec} {"m12"}

MACRO {sty} {"m01"}

MACRO {lut} {"m02"}


MACRO {kwi} {"m04"}

MACRO {maj} {"m05"}

MACRO {cze} {"m06"}

MACRO {lip} {"m07"}

MACRO {sie} {"m08"}

MACRO {wrz} {"m09"}

MACRO {paz} {"m10"}

MACRO {lis} {"m11"}

MACRO {gru} {"m12"}





ENTRY
  { address
    author
    booktitle
    chapter
    edition
    editor
    howpublished
    institution
    journal
    key
    month
    note
    number
    organization
    pages
    publisher
    school
    series
    title
    type
    volume
    year
    url
    lang
  }
  {}

  { label }



INTEGERS { output.state before.all mid.sentence after.sentence after.block }

FUNCTION {init.state.consts}
{ #0 'before.all :=
  #1 'mid.sentence :=
  #2 'after.sentence :=
  #3 'after.block :=
}


STRINGS { s t }




FUNCTION {not}
{   { #0 }
    { #1 }
  if$
}

FUNCTION {and}
{   'skip$
    { pop$ #0 }
  if$
}

FUNCTION {or}
{   { pop$ #1 }
    'skip$
  if$
}




STRINGS { chosenword }


% Parametry (od lewej): 1 - polskie slowo
%                       2 - angielski odpowiednik
%                       3 - niemiecki odpowiednik
%
% W tej wersji na sztywno wybierana jest wersja
% angielskojezyczna.
%
FUNCTION {BK.choose.word}
{
  pop$
  'chosenword :=
  pop$
  chosenword
}



FUNCTION {BK.make.first.upper}
{
  "u" change.case$ "t" change.case$
}



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Poczatek:  Zestawienie tlumaczen slow (pl, en, de) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


FUNCTION {BK.output.word.volume}
{
 "wolumen" "Vol." "Band" BK.choose.word
}


FUNCTION {BK.output.word.series}
{
 "serii" "series" "der Reihe" BK.choose.word
}


FUNCTION {BK.output.word.number}
{
 "numer" "No." "Nummer" BK.choose.word
}


FUNCTION {BK.output.word.chapter}
{
 "rozdziaÄšÂ" "chapter" "Kapitel" BK.choose.word
}


FUNCTION {BK.output.word.treport}
{
 "Raport instytutowy" "Technical Report" 
                      "Technischer Bericht" BK.choose.word
}


FUNCTION {BK.output.word.crossref}
{
  "Patrz" "In" "In" BK.choose.word
}


FUNCTION {BK.output.word.crossrefsp}
{
  "Patrz " "In " "In " BK.choose.word
}


FUNCTION {BK.output.word.and}
{
  " i~" " and~" "und~" BK.choose.word
}


FUNCTION {BK.output.word.mastersthesis}
{
 "Praca magisterska" "Master's thesis" 
                      "Diplomarbeit" BK.choose.word
}


FUNCTION {BK.output.word.phdthesis}
{
 "Praca doktorska" "PhD thesis" "Doktorarbeit" BK.choose.word
}


FUNCTION {BK.output.word.wydanie}
{
 "wydanie " "" "" BK.choose.word
}


FUNCTION {BK.output.word.edition}
{
 "" " edition" "Au\discretionary{f-}{l}{f\null l}age" 
                                        BK.choose.word
}


%
% Tlumaczenia nazw miesiecy
%

FUNCTION {BK.output.word.January}
{
 "StyczeÄšÂ" "January" "Januar" BK.choose.word
}

FUNCTION {BK.output.word.February}
{
 "Luty" "February" "Februar" BK.choose.word
}

FUNCTION {BK.output.word.March}
{
 "Marzec" "March" "M\xumlaut rz" BK.choose.word
}

FUNCTION {BK.output.word.April}
{
 "KwiecieÄšÂ" "April" "April" BK.choose.word
}

FUNCTION {BK.output.word.May}
{
 "Maj" "May" "Mai" BK.choose.word
}

FUNCTION {BK.output.word.June}
{
 "Czerwiec" "June" "Juni" BK.choose.word
}

FUNCTION {BK.output.word.July}
{
 "Lipiec" "July" "Juli" BK.choose.word
}

FUNCTION {BK.output.word.August}
{
 "SierpieÄšÂ" "August" "August" BK.choose.word
}

FUNCTION {BK.output.word.September}
{
 "WrzesieÄšÂ" "September" "September" BK.choose.word
}

FUNCTION {BK.output.word.October}
{
 "PaÄšĹdziernik" "October" "Oktober" BK.choose.word
}

FUNCTION {BK.output.word.November}
{
 "Listopad" "November" "November" BK.choose.word
}

FUNCTION {BK.output.word.December}
{
 "GrudzieÄšÂ" "December" "Dezember" BK.choose.word
}


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Koniec:  Zestawienie tlumaczen slow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%
%  Dodaje spacje na koncu i poczatku lancucha, ktory jest 
%  na stosie
%
FUNCTION {BK.add.spaces.both}				       
{
  's :=
  " " s * " " *
}

% Dodaje slowko "serii"
% otoczone spacjami
FUNCTION {BK.output.spaced.series}
{
  BK.output.word.series
  BK.add.spaces.both
}

FUNCTION {BK.output.word.u.number}
{
  BK.output.word.number
  BK.make.first.upper
}

FUNCTION {BK.output.word.u.volume}
{
  BK.output.word.volume
  BK.make.first.upper
}






FUNCTION {BK.output.month}
{
 duplicate$
 "m01" =
  { pop$ BK.output.word.January }
  { duplicate$
   "m02" =
   { pop$ BK.output.word.February }
   { duplicate$
    "m03" = 
    { pop$ BK.output.word.March }
    { duplicate$
     "m04" = 
     { pop$ BK.output.word.April }
     { duplicate$
      "m05" = 
      { pop$ BK.output.word.May }
      { duplicate$
       "m06" = 
       { pop$ BK.output.word.June }
       { duplicate$
        "m07" = 
        { pop$ BK.output.word.July }
        { duplicate$
          "m08" = 
          { pop$ BK.output.word.August }
          { duplicate$
           "m09" = 
           { pop$ BK.output.word.September }
           { duplicate$
            "m10" = 
            { pop$ BK.output.word.October }
            { duplicate$
             "m11" = 
             { pop$ BK.output.word.November }
             { duplicate$
              "m12" = 
              { pop$ BK.output.word.December }
              'skip$
              if$
            } if$
           } if$
          } if$
         } if$
        } if$
       } if$
      } if$
     } if$
    } if$ 
   } if$
  } if$
}



FUNCTION {output.nonnull}
{ 's :=
  s "" =
  'skip$
 {
  output.state mid.sentence =
    { ", " * write$ }
    { output.state after.block =
	{ add.period$ write$
	  newline$
	  "\newblock " write$
	}
	{ output.state before.all =
	    'write$
	    { add.period$ " " * write$ }
	  if$
	}
      if$
      mid.sentence 'output.state :=
    }
  if$
  s
 } if$
}



FUNCTION {BK.output.nonnull.without.period} %BK
{ 's :=
  output.state mid.sentence =
    { " " * write$ }               % Normalnie jest tu ", "
    { output.state after.block =
	{ add.period$ write$
	  newline$
	  "\newblock " write$
	}
	{ output.state before.all =
	    'write$
	    { add.period$ " " * write$ }
	  if$
	}
      if$
      mid.sentence 'output.state :=
    }
  if$
  s
}



FUNCTION {BK.output.nonnull.with.dot} %BK
{ 
  's :=
  output.state mid.sentence =
    { ". " * write$ }               % Normalnie jest tu ", "
    {  output.state after.block =
	{ ". " * write$   %add.period
	  newline$
	  "\newblock " write$
	}
	{ output.state before.all =
	    'write$
	    {  ". " * write$ } %add.period$
	  if$
	}
      if$
      mid.sentence 'output.state :=
    }
  if$
  s
}




FUNCTION {BK.output.url.nonnull}
{ 's :=
  output.state mid.sentence =
    { ", " * write$ }
    { output.state after.block =
	{ % add.period$
          ", " * write$
	  newline$
	  "\newblock " write$
	}
	{
          'write$
         % output.state before.all =
	 %   'write$
	 %   { add.period$ " " * write$ }
	 % if$
	}
      if$
      mid.sentence 'output.state :=
    }
  if$
  s
}





FUNCTION {output}
{ duplicate$ empty$
    'pop$
    'output.nonnull
  if$
}


FUNCTION {BK.output.with.dot}
{ duplicate$ empty$
    'pop$
    'BK.output.nonnull.with.dot
  if$
}



FUNCTION {BK.outputurl}
{ duplicate$ empty$
    'pop$
    'BK.output.url.nonnull
  if$
}


FUNCTION {output.check}
{ 't :=
  duplicate$ empty$
    { pop$ "empty " t * " in " * cite$ * warning$ }
    'output.nonnull
  if$
}


FUNCTION {BK.output.check.without.period}
{ 't :=
  duplicate$ empty$
    { pop$ "empty " t * " in " * cite$ * warning$ }
    'BK.output.nonnull.without.period
  if$
}



FUNCTION {BK.output.check.with.dot}
{ 't :=
  duplicate$ empty$
    { pop$ "empty " t * " in " * cite$ * warning$ }
    'BK.output.nonnull.with.dot
  if$
}




FUNCTION {output.bibitem}
{ newline$

  "\bibitem{" write$

  cite$ write$
  "}" write$
  newline$
  ""
  before.all 'output.state :=
}


FUNCTION {fin.entry}
{ add.period$
  write$
  newline$
}

% BK
FUNCTION {fin.website.entry}
{% add.period$
  write$
  newline$
}



FUNCTION {new.block}
{ output.state before.all =
    'skip$
    { after.block 'output.state := }
  if$
}


FUNCTION {new.block.checka}
{ empty$
    'skip$
    'new.block
  if$
}




FUNCTION {new.sentence}
{ output.state after.block =
    'skip$
    { output.state before.all =
	'skip$
	{ after.sentence 'output.state := }
      if$
    }
  if$
}



FUNCTION {new.block.checkb}
{ empty$
  swap$ empty$
  and
    'skip$
    'new.block
  if$
}


FUNCTION {new.sentence.checka}
{ empty$
    'skip$
    'new.sentence
  if$
}

FUNCTION {new.sentence.checkb}
{ empty$
  swap$ empty$
  and
    'skip$
    'new.sentence
  if$
}


FUNCTION {field.or.null}
{ duplicate$ empty$
    { pop$ "" }
    'skip$
  if$
}

FUNCTION {emphasize}
{ duplicate$ empty$
    { pop$ "" }
    { "{\em " swap$ * "}" * }
  if$
}



INTEGERS { nameptr namesleft numnames }




FUNCTION {BK.format.names}
{ 's :=
  #1 'nameptr :=
  s num.names$ 'numnames :=

 numnames #3 >
 {
  numnames 'namesleft :=

      s nameptr "{f.~}{vv~}{ll}{, jj}" format.name$ 't :=

      nameptr #1 >
        'skip$
	't
      if$
      " et~al" *
 }
 { 
  numnames 'namesleft :=
    { namesleft #0 > }

    { s nameptr "{f.~}{vv~}{ll}{, jj}" format.name$ 't :=

      nameptr #1 >
	{ numnames #2 =   %BL poprawka 22.10.97 
	    { ", " * }    % 
	    'skip$        %
	 if$              %
         namesleft #1 >
	    { ", " * t * }
	    { numnames #2 >
		{ "," * }
		'skip$
	      if$
	      t "others" =
                { " et~al" * }  %BK bylo " i~in."
                { " " * t * }   %BL poprawka 9.10.97
              if$
	    }
	  if$
	}
	't
      if$
      nameptr #1 + 'nameptr :=
      namesleft #1 - 'namesleft :=
    }
  while$
 }
 if$
}

FUNCTION {format.authors}
{ author empty$
    { "" }
    { author BK.format.names }
  if$
}

FUNCTION {format.editors}
{ editor empty$
    { "" }
    { % editor num.names$ #1 >
      %  { " Red. " * }    %BK
      %  { " Red. " * }    %BK
      % if$
      " Red. " editor BK.format.names * %BK
    }
  if$
}


FUNCTION {format.title}
{ title empty$
    { "" }

    { title "t" change.case$ }

  if$
}



FUNCTION {BK.format.website.title}
{ title empty$
    { "" }

    { title emphasize }

  if$
}



FUNCTION {n.dashify}
{ 't :=
  ""
    { t empty$ not }
    { t #1 #1 substring$ "-" =
	{ t #1 #2 substring$ "--" = not
	    { "--" *
	      t #2 global.max$ substring$ 't :=
	    }
	    {   { t #1 #1 substring$ "-" = }
		{ "-" *
		  t #2 global.max$ substring$ 't :=
		}
	      while$
	    }
	  if$
	}
	{ t #1 #1 substring$ *
	  t #2 global.max$ substring$ 't :=
	}
      if$
    }
  while$
}


FUNCTION {format.date}
{ year empty$
    { month empty$
	{ "" }
	{ "there's a month but no year in " cite$ * warning$
	  month BK.output.month % BK dodane output.month
	}
      if$
    }
    { month empty$
	'year
	{ month BK.output.month ", " * year * }  
                           %BK  wstawiony ", " zamiast " "
      if$
    }
  if$
}


FUNCTION {format.btitle}
{ title emphasize
  add.period$
}


FUNCTION {tie.or.space.connect}
{ duplicate$ text.length$ #3 <
    { "~" }
    { " " }
  if$
  swap$ * *
}


FUNCTION {either.or.check}
{ empty$
    'pop$
    { "can't use both " swap$ * " fields in " * cite$ * warning$ }
  if$
}


FUNCTION {format.bvolume}
{ volume empty$
    { "" }
    { BK.output.word.volume volume tie.or.space.connect
      series empty$
	'skip$
        { BK.output.spaced.series * series emphasize * }
      if$
      "volume and number" number either.or.check
    }
  if$
}


FUNCTION {format.number.series}
{ volume empty$
    { number empty$
	{ series field.or.null }
	{ output.state mid.sentence =
            { BK.output.word.number }     % BK. bylo "number"
	    { BK.output.word.u.number }   % BK. bylo "Number"
	  if$
	  number tie.or.space.connect
	  series empty$
	    { "there's a number but no series in " cite$ * warning$ }
            { BK.output.spaced.series * series * }
	  if$
	}
      if$
    }
    { "" }
  if$
}


FUNCTION {format.edition}
{ edition empty$
    { "" }
    { output.state mid.sentence =
        { BK.output.word.wydanie edition "l" change.case$ * 
           BK.output.word.edition * }
        { BK.output.word.wydanie edition "t" change.case$ * 
           BK.output.word.edition * }
      if$
    }
  if$
}


INTEGERS { multiresult }

FUNCTION {multi.page.check}
{ 't :=
  #0 'multiresult :=
    { multiresult not
      t empty$ not
      and
    }
    { t #1 #1 substring$
      duplicate$ "-" =
      swap$ duplicate$ "," =
      swap$ "+" =
      or or
	{ #1 'multiresult := }
	{ t #2 global.max$ substring$ 't := }
      if$
    }
  while$
  multiresult
}


FUNCTION {format.pages}
{ pages empty$
    { "" }
    { pages multi.page.check
        { "pp." pages n.dashify tie.or.space.connect } %BK
        { "pp." pages tie.or.space.connect }           %BK
      if$
    }
  if$
}


FUNCTION {format.vol.num.pages}
{ 
  volume empty$
   { volume field.or.null }
   { BK.output.word.volume  " " * volume * 
     number empty$
     { pages empty$
       { ""   }
       { ", " }
       if$
     }
     { ", " }
     if$ 
     *
   } % field.or.null
  if$
  number empty$
    'skip$
    { BK.output.word.number " " * number *
      pages empty$
       { "" }
       { ", " }
      if$
      * *
      volume empty$
	{ "there's a number but no volume in " cite$ * warning$ }
	'skip$
      if$ } if$ pages empty$ 'skip$ { duplicate$ empty$ { pop$
    format.pages } { " pp. " * pages n.dashify * } if$ } if$ }



FUNCTION {format.chapter.pages}
{ chapter empty$
    'format.pages
    { type empty$
        { BK.output.word.chapter }                 %problem
	{ type "l" change.case$ }
      if$
      chapter tie.or.space.connect
      pages empty$
	'skip$
	{ ", " * format.pages * }
      if$
    }
  if$
}


FUNCTION {format.in.proceedings.booktitle}
{ booktitle empty$
    { "" }
    { editor empty$
        { "In: " booktitle * ". " * " Proceedings" emphasize * }   %BK
        { "In: " booktitle * ". " * " Proceedings" emphasize * format.editors *  }  %BK
      if$
    }
  if$
}



FUNCTION {format.in.ed.booktitle}
{ booktitle empty$
    { "" }
    { editor empty$
        { "In: " booktitle emphasize * }              
        { "In: " booktitle emphasize *  format.editors *  }
      if$
    }
  if$
}


FUNCTION {empty.misc.check}
{ author empty$ title empty$ howpublished empty$
  month empty$ year empty$ note empty$
  and and and and and

  key empty$ not and

    { "all relevant fields are empty in " cite$ * warning$ }
    'skip$
  if$
}


FUNCTION {format.thesis.type}
{ type empty$
    'skip$
    { pop$
      type "t" change.case$
    }
  if$
}


FUNCTION {format.tr.number}
{ type empty$
    { BK.output.word.treport }
    'type
  if$
  number empty$
    { "t" change.case$ }
    { number tie.or.space.connect }
  if$
}


FUNCTION {format.article.crossref}
{ key empty$
    { journal empty$
	{ "need key or journal for " cite$ * " to crossref " * crossref *
	  warning$
	  ""
	}
        { BK.output.word.crossref 
              " {\em " * journal * "\/}" * 
        }
      if$
    }
    { BK.output.word.crossref " " * key * }
  if$
  " \cite{" * crossref * "}" *
}


FUNCTION {format.crossref.editor}
{ editor #1 "{vv~}{ll}" format.name$
  editor num.names$ duplicate$
  #2 >
    { pop$ " et~al." * }
    { #2 <
	'skip$
	{ editor #2 "{f. }{vv }{ll}{ jj}" format.name$ "others" =
            { " et~al." * }
            { BK.output.word.and * editor #2 "{vv~}{ll}" 
                                                 format.name$ * }
	  if$
	}
      if$
    }
  if$
}


FUNCTION {format.book.crossref}
{ volume empty$
    { "empty volume in " cite$ * "'s crossref of " * 
                                        crossref * warning$
      BK.output.word.crossref
    }
    { BK.output.word.u.volume  volume tie.or.space.connect
      BK.output.spaced.series *
    }
  if$
  editor empty$
  editor field.or.null author field.or.null =
  or
    { key empty$
	{ series empty$
	    { "need editor, key, or series for " cite$ * " to crossref " *
	      crossref * warning$
	      "" *
	    }
	    { "{\em " * series * "\/}" * }
	  if$
	}
	{ key * }
      if$
    }
    { format.crossref.editor * }
  if$
  " \cite{" * crossref * "}" *
}


FUNCTION {format.incoll.inproc.crossref}
{ editor empty$
  editor field.or.null author field.or.null =
  or
    { key empty$
	{ booktitle empty$
	    { "need editor, key, or booktitle for " cite$ * " to crossref " *
	      crossref * warning$
	      ""
	    }
            { BK.output.word.crossref " {\em " * 
                                         booktitle * "\/}" * }
	  if$
	}
        { BK.output.word.crossrefsp key * }
      if$
    }
    { BK.output.word.crossrefsp format.crossref.editor * }
  if$
  " \cite{" * crossref * "}" *
}


FUNCTION {article}
{ output.bibitem
  format.authors "author" output.check
  new.block
  format.title "title" output.check
  new.block
  crossref missing$
    { journal "journal" output.check  % BK usunieto emphasize
      format.date "year" output.check
      format.vol.num.pages output
    }
    { format.article.crossref output.nonnull
      format.pages output
    }
  if$
  new.block
  note output
  fin.entry
}

FUNCTION {book}  % BK zmodyfikowana
{ output.bibitem
  author empty$
    { format.btitle "title" output.check
      new.block
      format.editors "author and editor" output.check }
    { format.authors output.nonnull
      crossref missing$
	{ "author and editor" editor either.or.check }
	'skip$
      if$
      new.block
      format.btitle "title" output.check
    }
  if$
  crossref missing$
    { format.bvolume output
      new.block
      format.number.series output
      new.sentence
      address output
      format.edition output
      publisher "publisher" output.check
      format.date "year" BK.output.check.without.period
    }
    { new.block
      format.book.crossref output.nonnull
      format.edition output
      format.date "year" output.check
    }
  if$
  new.block
  note output
  fin.entry
}


FUNCTION {booklet}
{ output.bibitem
  format.authors output
  new.block
  format.title "title" output.check
  howpublished address new.block.checkb
  howpublished output
  address output
  format.date output
  new.block
  note output
  fin.entry
}



FUNCTION {inbook}
{ output.bibitem
  author empty$
    { format.editors "author and editor" output.check }
    { format.authors output.nonnull
      crossref missing$
	{ "author and editor" editor either.or.check }
	'skip$
      if$
    }
  if$
  new.block
  format.btitle "title" output.check
  crossref missing$
    { 
      new.sentence
      address output
      publisher "publisher" output.check
      format.date "year" BK.output.check.without.period
      format.edition output
      format.bvolume output
      format.chapter.pages "chapter and pages" output.check
      new.block
      format.number.series output
    }
    { 
      format.edition output
      format.date "year" output.check
      format.chapter.pages "chapter and pages" output.check
      new.block
      format.book.crossref output.nonnull
    }
  if$
  new.block
  note output
  fin.entry
}


FUNCTION {incollection}
{ output.bibitem
  format.authors "author" output.check
  new.block
  format.title "title" output.check
  new.block
  crossref missing$
    { format.in.ed.booktitle "booktitle" output.check
      format.bvolume output
      format.number.series output
      format.chapter.pages output
      new.sentence
      format.edition output
      address output
      publisher "publisher" output.check
      format.date "year" BK.output.check.without.period
    }
    { format.incoll.inproc.crossref output.nonnull
      format.chapter.pages output
    }
  if$
  new.block
  note output
  fin.entry
}


FUNCTION {inproceedings}
{ output.bibitem
  format.authors "author" output.check
  new.block
  format.title "title" output.check
  new.block
  crossref missing$
    { format.in.proceedings.booktitle "booktitle"
                            BK.output.check.with.dot
      address empty$
	{ organization publisher new.sentence.checkb
	  organization output
	  publisher output
	}
	{ address output.nonnull
        %	  format.date "year" output.check
	%  new.sentence
	  organization output
	  publisher output
	}
      if$
      format.date  "year" output.check
      format.bvolume BK.output.with.dot
      format.number.series output
      format.pages output
    }
    { format.incoll.inproc.crossref output.nonnull
      format.pages output
    }
  if$
  new.block
  note output
  fin.entry
}


FUNCTION {conference} { inproceedings }


FUNCTION {manual}
{ output.bibitem
  author empty$
    { organization empty$
	'skip$
	{ organization output.nonnull
	  address output
	}
      if$
    }
    { format.authors output.nonnull }
  if$
  new.block
  format.btitle "title" output.check
  author empty$
    { organization empty$
	{ address new.block.checka
	  address output
	}
	'skip$
      if$
    }
    { organization address new.block.checkb
      organization output
      address output
    }
  if$
  format.edition output
  format.date output
  new.block
  note output
  fin.entry
}


FUNCTION {mastersthesis}
{ output.bibitem
  format.authors "author" output.check
  new.block
  format.title "title" output.check
  new.block
  BK.output.word.mastersthesis format.thesis.type output.nonnull
  school "school" output.check
  address output
  format.date "year" output.check
  new.block
  note output
  fin.entry
}


FUNCTION {misc}
{ output.bibitem
  format.authors output
  title howpublished new.block.checkb
  format.title output
  newline$
  howpublished new.block.checka
  howpublished output
  format.date output
  new.block
  note output
  fin.entry
  empty.misc.check
}



FUNCTION {webside}
{ output.bibitem
  format.authors output
  title url new.block.checkb
  BK.format.website.title output
  url new.block.checka
  url BK.outputurl
  "\\" write$  % BK
  format.date output
  new.block
%  note output
  fin.website.entry
  empty.misc.check
}


FUNCTION {website}
{ output.bibitem
  format.authors output
  title url new.block.checkb
  BK.format.website.title output
  url new.block.checka
  url BK.outputurl
  "\\" write$  % BK
  format.date output
  new.block
%  note output
  fin.website.entry
  empty.misc.check
}




FUNCTION {phdthesis}
{ output.bibitem
  format.authors "author" output.check
  new.block
  format.btitle "title" output.check
  new.block
  BK.output.word.phdthesis format.thesis.type output.nonnull
  school "school" output.check
  address output
  format.date "year" output.check
  new.block
  note output
  fin.entry
}


FUNCTION {proceedings}
{ output.bibitem
  editor empty$
    { organization output }
    { format.editors output.nonnull }
  if$
  new.block
  format.btitle "title" output.check
  format.bvolume output
  format.number.series output
  address empty$
    { editor empty$
	{ publisher new.sentence.checka }
	{ organization publisher new.sentence.checkb
	  organization output
	}
      if$
      publisher output
      format.date "year" output.check
    }
    { address output.nonnull
      format.date "year" output.check
      new.sentence
      editor empty$
	'skip$
	{ organization output }
      if$
      publisher output
    }
  if$
  new.block
  note output
  fin.entry
}


FUNCTION {techreport}
{ output.bibitem
  format.authors "author" output.check
  new.block
  format.title "title" output.check
  new.block
  format.tr.number output.nonnull
  institution "institution" output.check
  address output
  format.date "year" output.check
  new.block
  note output
  fin.entry
}


FUNCTION {unpublished}
{ output.bibitem
  format.authors "author" output.check
  new.block
  format.title "title" output.check
  new.block
  note "note" output.check
  format.date output
  fin.entry
}


FUNCTION {default.type} { misc }










MACRO {acmcs} {"ACM Computing Surveys"}

MACRO {acta} {"Acta Informatica"}

MACRO {cacm} {"Communications of the ACM"}

MACRO {ibmjrd} {"IBM Journal of Research and Development"}

MACRO {ibmsj} {"IBM Systems Journal"}

MACRO {ieeese} {"IEEE Transactions on Software Engineering"}

MACRO {ieeetc} {"IEEE Transactions on Computers"}

MACRO {ieeetcad}
 {"IEEE Transactions on Computer-Aided Design of Integrated Circuits"}

MACRO {ipl} {"Information Processing Letters"}

MACRO {jacm} {"Journal of the ACM"}

MACRO {jcss} {"Journal of Computer and System Sciences"}

MACRO {scp} {"Science of Computer Programming"}

MACRO {sicomp} {"SIAM Journal on Computing"}

MACRO {tocs} {"ACM Transactions on Computer Systems"}

MACRO {tods} {"ACM Transactions on Database Systems"}

MACRO {tog} {"ACM Transactions on Graphics"}

MACRO {toms} {"ACM Transactions on Mathematical Software"}

MACRO {toois} {"ACM Transactions on Office Information Systems"}

MACRO {toplas} {"ACM Transactions on Programming Languages and Systems"}

MACRO {tcs} {"Theoretical Computer Science"}

MACRO {gust} {"Grupa UÄšĹşytkownikÄĹw Systemu \TeX"}




READ




FUNCTION {sortify}
{ purify$
  "l" change.case$
}

INTEGERS { len }

FUNCTION {chop.word}
{ 's :=
  'len :=
  s #1 len substring$ =
    { s len #1 + global.max$ substring$ }
    's
  if$
}









FUNCTION {sort.format.names}
{ 's :=
  #1 'nameptr :=
  ""
  s num.names$ 'numnames :=
  numnames 'namesleft :=
    { namesleft #0 > }
    { nameptr #1 >
	{ "   " * }
	'skip$
      if$

      s nameptr "{vv{ } }{ll{ }}{  ff{ }}{  jj{ }}" format.name$ 't :=

      nameptr numnames = t "others" = and
        { "et~al." * }
	{ t sortify * }
      if$
      nameptr #1 + 'nameptr :=
      namesleft #1 - 'namesleft :=
    }
  while$
}


FUNCTION {sort.format.title}
{ 't :=
  "A " #2
    "An " #3
      "The " #4 t chop.word
    chop.word
  chop.word
  sortify
  #1 global.max$ substring$
}



FUNCTION {author.sort}
{ author empty$
    { key empty$
	{ "to sort, need author or key in " cite$ * warning$
	  ""
	}
	{ key sortify }
      if$
    }
    { author sort.format.names }
  if$
}

% To trzeba zmienic
STRINGS { httpsortkey }

FUNCTION {website.sort}
{ author empty$
    { key empty$
	{ "to sort, need author or key in " cite$ * warning$
	  ""
	}
	{ key sortify }
      if$
    }
    { 
       url empty$
       { author sort.format.names }
       {
         "ZZZ" 'httpsortkey :=
         "ZZZ" author * 'httpsortkey :=
         httpsortkey sortify
       }
      if$

    }
  if$
}




FUNCTION {editor.organization.sort}
{ editor empty$
    { organization empty$
	{ key empty$
	    { "to sort, need editor, organization, or key in " cite$ * warning$
	      ""
	    }
	    { key sortify }
	  if$
	}
	{ "The " #4 organization chop.word sortify }
      if$
    }
    { editor sort.format.names }
  if$
}



FUNCTION {author.editor.sort}
{ author empty$
    { editor empty$
	{ key empty$
	    { "to sort, need author, editor, or key in " cite$ * warning$
	      ""
	    }
	    { key sortify }
	  if$
	}
	{ editor sort.format.names }
      if$
    }
    { author sort.format.names }
  if$
}


FUNCTION {author.title.sort}
{ author empty$
    { editor empty$
	{ key empty$
	    { "to sort, need author, editor, or key in " cite$ * warning$
	      ""
	    }
	    { key sortify }
	  if$
	}
        { title  sortify }
	%{ editor sort.format.names }
      if$
    }
    { author sort.format.names }
  if$
}




FUNCTION {author.organization.sort}
{ author empty$
    { organization empty$
	{ key empty$
	    { "to sort, need author, organization, or key in " cite$ * warning$
	      ""
	    }
	    { key sortify }
	  if$
	}
	{ "The " #4 organization chop.word sortify }
      if$
    }
    { author sort.format.names }
  if$
}



FUNCTION {presort}

{ 
  type$ "book" =

  type$ "inbook" =
  or
    'author.title.sort         % BK  Bylo author.editor.sort
    { type$ "proceedings" =
	'editor.organization.sort
	{ type$ "manual" =
	    'author.organization.sort
            { type$ "website" =  
              'website.sort
              { type$ "webside" =
               'website.sort
               'author.sort
               if$
              }
             if$
            }
	  if$
	}
      if$
    }
  if$

  "    "
  *
  year field.or.null sortify
  *
  "    "
  *
  title field.or.null
  sort.format.title
  *
  #1 entry.max$ substring$
  'sort.key$ :=
}

ITERATE {presort}


SORT







STRINGS { longest.label }

INTEGERS { number.label longest.label.width }

FUNCTION {initialize.longest.label}
{ "" 'longest.label :=
  #1 'number.label :=
  #0 'longest.label.width :=
}

FUNCTION {longest.label.pass}
{ number.label int.to.str$ 'label :=
  number.label #1 + 'number.label :=
  label width$ longest.label.width >
    { label 'longest.label :=
      label width$ 'longest.label.width :=
    }
    'skip$
  if$
}

EXECUTE {initialize.longest.label}

ITERATE {longest.label.pass}




FUNCTION {begin.bib}

{ preamble$ empty$

    'skip$
    { preamble$ write$ newline$ }
  if$
  "\begin{thebibliography}{"  longest.label  * "}" * write$ newline$
}

EXECUTE {begin.bib}

EXECUTE {init.state.consts}


ITERATE {call.type$}


FUNCTION {end.bib}
{ newline$
  "\end{thebibliography}" write$ newline$
}

EXECUTE {end.bib}

