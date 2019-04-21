const fs = require('fs');
const path = require('path');
const readline = require('readline');

const rl = readline.createInterface({
  input: process.stdin,
  output: process.stdout
});

const sd_path = process.argv[2] || '/media/ungato/ELLIOT2';

console.log('SD Card Recovery Tool');

function dumpHistory(history, dontShowName) {
  if(history.children[0]) {
    dumpHistory(history.children[0], true);
  } else {
    console.log(`Content of history for file ${history.name}.`);
  }
  const life = history.content.lifetime ? (history.content.lifetime / 1000.0).toFixed(3) : '<Not Set>';
  console.log(`Filename: ${history.name}\tLife: ${life} seconds\tSize: ${JSON.stringify(history.content).length}`);
  if(!dontShowName) {
    console.log('');
  }
}

//Credit to https://stackoverflow.com/questions/18017869/build-tree-array-from-flat-array-in-javascript
function list_to_tree(list) {
  var map = {}, node, roots = [], i;
  for (i = 0; i < list.length; i += 1) {
      map[list[i].name] = i; // initialize the map
      list[i].children = []; // initialize the children
  }
  for (i = 0; i < list.length; i += 1) {
      node = list[i];
      if (node.content.prevName && map[node.content.prevName]) {
          list[map[node.content.prevName.substr(5)]].children.push(node);
      } else {
          roots.push(node);
      }
  }
  return roots;
}

Array.prototype.removeIf = function(callback) {
  var i = this.length;
  while (i--) {
      if (callback(this[i], i)) {
          this.splice(i, 1);
      }
  }
};

function printCommonUsage() {
  console.log('To switch to a file, type');
  console.log('    switch <filename>');
  console.log('To dump the content of a file, type');
  console.log('    dump <filename>');
  console.log('To reprint the list of histories, type');
  console.log('    history');
  console.log('To rename a save, type');
  console.log('    rename <prevname> <newname>');
}

function init() {
  const files = [];
  const filenames = fs.readdirSync(sd_path);
  for(let filename of filenames) {
    if(filename !== 'latest.txt') {
      files.push({
        content: JSON.parse(fs.readFileSync(path.join(sd_path, filename))),
        name: filename
      });
    }
  }

  let histories = list_to_tree(files);

  console.log(`Found ${histories.length} unique histories.`);
  for(let history of histories) {
    dumpHistory(history);
  }

  printCommonUsage();

  rl.on('line', line => {
    if(line.startsWith('switch')) {
      const nameGiven = line.substr(7);
      let f = files.find(o => o.name.startsWith(nameGiven));
      if(!f) {
        console.log('No such file to switch to.');
      } else {
        fs.writeFileSync(path.join(sd_path, 'latest.txt'), '/usd/' + f.name);
      }
    }
    if(line.startsWith('dump')) {
      let nameGiven;
      if(line.length === 4) {
        nameGiven = fs.readFileSync(path.join(sd_path, 'latest.txt'), 'UTF-8').substr(5);
      } else {
        nameGiven = line.substr(5);
      }
      let f = files.find(o => o.name.startsWith(nameGiven));
      if(!f) {
        console.log('No such file to dump.');
      } else {
        console.log(JSON.stringify(JSON.parse(fs.readFileSync(path.join(sd_path, f.name), 'UTF-8')), null, 2));
      }
    }
    if(line.startsWith('rename')) {
      const parts = line.substr(7).split(' ');
      const orig = parts[0];
      const target = parts[1];
      let f = files.find(o => o.name.startsWith(orig));
      let old = f.name;
      //Move file
      fs.renameSync(path.join(sd_path, old), path.join(sd_path, target));
      //Change name of object
      f.name = target;
      //Change references in prevName
      for(let file of files) {
        if(file.content.prevName === '/usd/' + old) file.content.prevName = '/usd/' + target;
      }
      //Change reference in latest.txt
      if(fs.readFileSync(path.join(sd_path, 'latest.txt'), 'UTF-8').substr(5) === old) {
        fs.writeFileSync(path.join(sd_path, 'latest.txt'), '/usd/' + target);
      }
      //Recalculate tree
      histories = list_to_tree(files);
    }
    if(line.startsWith('history')) {
      console.log(`Found ${histories.length} unique histories.`);
      for(let history of histories) {
        dumpHistory(history);
      }
    }
  });
}


fs.exists(sd_path, exists => {
  if(exists) {
    init();
  } else {
    console.log(`SD card not inserted. Specify SD path as ${process.argv[0]} <path>.\n`);
    rl.close();
  }
});
