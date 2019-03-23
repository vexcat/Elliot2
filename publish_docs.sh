#!/bin/bash
doxygen elliot2-docs.cfg
mv html botdocs
rsync -avz botdocs root@ungato.tk:/var/www/html/ungato.tk/public_html/
mv botdocs html
