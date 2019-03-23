doxygen elliot2-docs.cfg
mv html botdocsnew
scp -r botdocsnew root@ungato.tk:/var/www/html/ungato.tk/public_html/
ssh root@ungato.tk rm -r /var/www/html/ungato.tk/public_html/botdocs
ssh root@ungato.tk mv /var/www/html/ungato.tk/public_html/botdocsnew /var/www/html/ungato.tk/public_html/botdocs
mv botdocsnew html
