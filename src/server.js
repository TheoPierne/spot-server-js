'use strict';

const { readdirSync, readFileSync } = require('node:fs');
const process = require('node:process');

/*
process.env['GRPC_VERBOSITY'] = 'debug';
process.env['GRPC_TRACE'] = 'all';
 */

const grpc = require('@grpc/grpc-js');

const { LoggerUtil } = require('./loggerUtil');

const logger = LoggerUtil.getLogger('SERVER');

const server = new grpc.Server();

readdirSync(`${__dirname}/server_services`)
  .filter(file => file.endsWith('.js'))
  .forEach(e => {
    const file = require(`${__dirname}/server_services/${e}`);
    server.addService(file.service, file.func);
  });

const keyPair = {
  private_key: readFileSync(`${__dirname}/resources/server.key`),
  cert_chain: readFileSync(`${__dirname}/resources/server.crt`)
}

server.bindAsync('0.0.0.0:443', grpc.ServerCredentials.createSsl(readFileSync(`${__dirname}/resources/ca.crt`), [keyPair], true), () => {
  logger.info(`Server started with ${server.handlers.size} services !`);
  server.start();
});

process.stdin.resume();
process.on('exit', exitHandler.bind(null, { cleanup: true }));
process.on('SIGINT', exitHandler.bind(null, { exit: true }));

function exitHandler(options) {
  if (options.cleanup) {
    logger.info('Server stopped !');
  }

  if (options.exit) {
    server.tryShutdown(() => {
      process.exit(0);
    });
  }
}
