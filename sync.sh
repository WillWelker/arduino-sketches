#!/bin/bash


# auto sync files to Github repo
# To add, commit and push all changes, issue:~$ ./sync.sh commit message up to 9 words

git add *
git commit -m "autosync $1 $2 $3 $4 $5 $6 $7 $9"
git push
