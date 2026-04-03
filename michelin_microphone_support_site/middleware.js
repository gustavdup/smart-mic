export const config = { matcher: '/:path*' };

export default function middleware(request) {
  const auth    = request.headers.get('authorization') ?? '';
  const [scheme, encoded] = auth.split(' ');

  if (scheme === 'Basic' && encoded) {
    const [user, pass] = atob(encoded).split(':');
    const validUser    = process.env.BASIC_AUTH_USER ?? 'michelin';
    const validPass    = process.env.BASIC_AUTH_PASS ?? '';

    if (validPass && user === validUser && pass === validPass) {
      return; // authorised — pass through to static file
    }
  }

  return new Response('Unauthorised', {
    status: 401,
    headers: { 'WWW-Authenticate': 'Basic realm="Smart Mic Tools", charset="UTF-8"' },
  });
}
